#include "fauna.h"

#include <set>
#include <algorithm>
#include <unordered_map>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <utility>
#include <thread>
#include <Windows.h>
#include <NIDAQmx.h>
#include "../_include/json.hpp"

#include "typedefFauna.h"
#include "tb_string.h"

namespace fs = std::filesystem;

using namespace std;
using ordered_json = nlohmann::ordered_json;

//====----====----====----====----====----====----====----====----VAL

#define FAUNA_LOCAL std::string(getenv("LOCALAPPDATA")) + "/Fauna/"
#define VERSIONINFO "Alpha v2.0.0"

//====----====----====----====----====----====----====----====----VAR

// LOCAL

atomic<int> myState;
unordered_map<string, STREAMPARAM> listStreamParam;
unordered_map<string, set<string>> listStreamChannel;

unordered_map<string, TaskHandle> listTaskHandle;
unordered_map<string, BUFFERINFO> listBufferInfo;
vector<thread> listThreadReport;

unordered_map<string, float64**> buffer_daq;

// MMAP

HANDLE handle_file_bufferState = 0;
HANDLE handle_mapping_bufferState = 0;
int* mmap_bufferState = 0;

//====----====----====----====----====----====----====----====----FDEC

//==== DAQ TASK
int _create_DAQTaskset();
int _clear_DAQTaskset();

//==== STREAM
int _bear_stream(int numMmapDev);
int _kill_stream();

int _create_bufferSystem();
int _clear_bufferSystem();

int _read_json_streamInfo(ordered_json& json_in, bool matchSerial);

int _write_json_offsetInfo();
int _init_mmap_bin_bufferState(int numMmapDev);

//==== ==== BUFFER
int __malloc_bufferSystem();
int __dalloc_bufferSystem();

int _init_mmap_bin_virtualBuffer(HANDLE* handle_file, HANDLE* handle_mapping, float64** viewer, int spb, int offsetDevice, int offsetChannel);

//==== Integrity
int _check_struct_streamParam(std::string& nameDevice, STREAMPARAM& streamParam);
int _check_json_streamInfo(ordered_json& json_in);

//==== THREAD
int _thread_report_buffer(std::string nameDevice);
int _thread_report_buffer_mmap(std::string nameDevice, int numMmapDev, int offsetDevice);

//==== INLINE
inline void _readAnalogF64(TaskHandle& taskHandle, int spb, float64 timeOut, int numChannel, float64* _buffer, int32* _numSamplesRead);

//====----====----====----====----====----====----====----====----FDEF

// DLL_EXPORT

int fauna_tell_versionInfo(std::string* _versionInfo)
{
  *_versionInfo = VERSIONINFO;
  return 0;
}

int fauna_tell_state(int* _state)
{
  *_state = myState;
  return 0;
}

int fauna_tell_listDevice(std::vector<std::string>* _listDevice)
{
  // Get full string
  int32 bufferSize = DAQmxGetSysDevNames(nullptr, 0);
  char* fullString = new char[bufferSize + 1] {};
  DAQmxGetSysDevNames(fullString, bufferSize);

  // Tokenize
  vector<string> temp_dev = {};
  _tokenize(fullString, ", ", &temp_dev);
  delete[] fullString;

  // Tell devices with analog inputs
  for (string dev : temp_dev)
  {
    vector<string> temp_chan;
    if (fauna_tell_listChannel(dev, &temp_chan) > 0)
    {
      _listDevice->push_back(dev);
    }
  }

  // Sort
  sort(_listDevice->begin(), _listDevice->end());

  return _listDevice->size();
}

int fauna_tell_listChannel(std::string& nameDevice, std::vector<std::string>* _listChannel)
{
  // Get full string
  int32 bufferSize = DAQmxGetDevAIPhysicalChans(nameDevice.c_str(), nullptr, 0);
  char* fullString = new char[bufferSize + 1] {};
  DAQmxGetDevAIPhysicalChans(nameDevice.c_str(), fullString, bufferSize);

  if (fullString == "")
  {
    return 0;
  }

  // Tokenize
  _tokenize(fullString, ", ", _listChannel);
  delete[] fullString;

  // Sort
  sort(_listChannel->begin(), _listChannel->end());

  return _listChannel->size();
}

int fauna_tell_rangeSps(std::string& nameDevice, double* _minSps, double* _maxSps)
{
  double min, max;

  if (DAQmxGetDevAIMinRate(nameDevice.c_str(), &min) != 0)
  {
    return -1;
  }

  if (DAQmxGetDevAIMaxSingleChanRate(nameDevice.c_str(), &max) != 0)
  {
    return -2;
  }

  *_minSps = min;
  *_maxSps = max;
  return 0;
}

int fauna_tell_listBias(std::string& nameDevice, std::vector<double>* _listBias)
{
  int32 bufferSize = 2;
  float64* buffer = new float64[bufferSize]{ 0 };
  while (DAQmxGetDevAIVoltageRngs(nameDevice.c_str(), buffer, bufferSize) != 0)
  {
    delete[] buffer;
    bufferSize *= 2;
    buffer = new float64[bufferSize];
  }

  for (int i = 0; i + 1 < bufferSize; i += 2)
  {
    double minVal = buffer[i];
    double maxVal = buffer[i + 1];

    if (minVal == 0.0 && maxVal == 0.0)
      break;

    if (std::abs(minVal) == std::abs(maxVal))
    {
      _listBias->push_back(maxVal);
    }
  }
  delete[] buffer;

  sort(_listBias->begin(), _listBias->end());
  return _listBias->size();
}

int fauna_tell_serialCode(std::string& nameDevice, std::string* _serialCode)
{
  vector<string> listDev;
  if (fauna_tell_listDevice(&listDev) <= 0)
  {
    return -1;
  }

  for (string dev : listDev)
  {
    if (dev == nameDevice)
    {
      uInt32 serial;
      DAQmxGetDevSerialNum(dev.c_str(), &serial);
      *_serialCode = to_string(serial);
      return 0;
    }
  }

  return -2;
}



int fauna_write_json_deviceInfo(const char* pathDir, const char* nameFile)
{
  vector<string> listDev;
  if (fauna_tell_listDevice(&listDev) <= 0)
  {
    return 1;
  }

  ofstream fout(string(pathDir) + string(nameFile) + ".json");

  if (fout.is_open() == false)
  {
    return -1;
  }

  ordered_json json_out;
  json_out["versionInfo"] = VERSIONINFO;

  ordered_json json_dev;
  json_dev["numDevice"] = listDev.size();

  for (string dev : listDev)
  {
    ordered_json json_each;

    string serial;
    if (fauna_tell_serialCode(dev, &serial) != 0)
    {
      return -0x0101;
    }

    double minSps, maxSps;
    if (fauna_tell_rangeSps(dev, &minSps, &maxSps) != 0)
    {
      return -0x0102;
    }
    
    vector<double> listBias;
    if (fauna_tell_listBias(dev, &listBias) <= 0)
    {
      return -0x0103;
    }

    vector<string> listChan;
    if (fauna_tell_listChannel(dev, &listChan) <= 0)
    {
      return -0x0104;
    }

    json_each["serialCode"] = serial;
    json_each["minSps"] = minSps;
    json_each["maxSps"] = maxSps;
    json_each["numBias"] = listBias.size();
    json_each["numChannel"] = listChan.size();
    json_each["listBias"] = listBias;
    json_each["listChannel"] = listChan;

    json_dev["listDevice"][dev] = json_each;
  }
  json_out["deviceInfo"] = json_dev;

  fout << json_out.dump(2);
  return 0;
}



int fauna_tell_listStreamDevice(std::vector<std::string>* _listDevice, std::vector<STREAMPARAM>* _listParam)
{
  if (listStreamParam.empty())
  {
    return -1;
  }

  _listDevice->clear();
  if (_listParam != NULL)
  {
    _listParam->clear();
  }

  // Pairing
  vector<pair<string, STREAMPARAM>> listPair;
  for (auto& dev : listStreamParam)
  {
    listPair.push_back({ dev.first, dev.second });
  }

  // Sorting
  sort
  (
    listPair.begin(),
    listPair.end(),
    [](const auto& a, const auto& b)
    {
      return a.first < b.first;
    }
  );

  // Division
  for (auto& dev : listPair)
  {
    _listDevice->push_back(dev.first);

    if (_listParam != NULL)
    {
      _listParam->push_back(dev.second);
    }
  }

  return _listDevice->size();
}

int fauna_tell_listStreamChannel(std::string& nameDevice, std::vector<std::string>* _listChannel)
{
  if (listStreamChannel.empty())
  {
    return 1;
  }

  for (auto& dev : listStreamChannel)
  {
    if (dev.first == nameDevice)
    {
      _listChannel->assign(dev.second.begin(), dev.second.end());
    }
  }

  sort(_listChannel->begin(), _listChannel->end());

  return _listChannel->size();
}



int fauna_do_insert_streamDevice(std::string& nameDevice, STREAMPARAM& streamParam, std::vector<std::string>& listChannel)
{
  // State Lock : READY
  if (myState != FAUNA_STATE_READY)
  {
    return -0xFFFF;
  }

  // Integrity Check
  int ret = _check_struct_streamParam(nameDevice, streamParam);
  if ((ret != 0) || (listChannel.empty() == true))
  {
    return -0xFF00;
  }

  // Param insertion
  if (listStreamParam.find(nameDevice) != listStreamParam.end())
  {
    ret += 0x0100;
  }
  listStreamParam.insert({ nameDevice, streamParam });

  // Channel insertion
  if (listStreamChannel.find(nameDevice) != listStreamChannel.end())
  {
    listStreamChannel[nameDevice].clear();
    ret += 0x0001;
  }
  set<string> orderedList(listChannel.begin(), listChannel.end());
  listStreamChannel.insert({nameDevice, orderedList});
  
  return ret;
}

int fauna_do_erase_streamDevice(std::string& nameDevice)
{
  // State Lock : READY
  if (myState != FAUNA_STATE_READY)
  {
    return -0xFFFF;
  }

  for (auto& dev : listStreamParam)
  {
    if (dev.first == nameDevice)
    {
      listStreamParam.erase(nameDevice);
      listStreamChannel.erase(nameDevice);
      return 0;
    }
  }

  return 1;
}

int fauna_do_clear_streamDevice()
{
  // State Lock : READY
  if (myState != FAUNA_STATE_READY)
  {
    return -0xFFFF;
  }

  if (listStreamParam.empty())
  {
    return 1;
  }

  listStreamParam.clear();
  listStreamChannel.clear();
  return 0;
}



int fauna_write_json_streamInfo(const char* pathDir, const char* nameFile)
{
  vector<string> listDev;
  vector<STREAMPARAM> listParam;

  if (fauna_tell_listStreamDevice(&listDev, &listParam) <= 0)
  {
    return -1;
  }

  ofstream fout(string(pathDir) + string(nameFile) + ".json");

  if (fout.is_open() == false)
  {
    return -1;
  }

  ordered_json json_out;
  json_out["versionInfo"] = VERSIONINFO;

  ordered_json json_dev;
  json_dev["numDevice"] = listDev.size();

  for (int i = 0; i < listDev.size(); i++)
  {
    ordered_json json_each;

    string serial;
    if (fauna_tell_serialCode(listDev[i], &serial) != 0)
    {
      return -0x0101;
    }

    vector<string> listChan;
    if (fauna_tell_listStreamChannel(listDev[i], &listChan) <= 0)
    {
      return -0x0102;
    }

    json_each["serialCode"] = serial;
    json_each["spb"] = listParam[i].spb;
    json_each["sps"] = listParam[i].sps;
    json_each["bias"] = listParam[i].bias;
    json_each["numChannel"] = listChan.size();
    json_each["listChannel"] = listChan;
    json_each["fileExport"] = listParam[i].fileStream;

    json_dev["listDevice"][listDev[i]] = json_each;
  }
  json_out["deviceInfo"] = json_dev;

  fout << json_out.dump(2);
  return 0;
}

int fauna_read_json_streamInfo(const char* pathFile, bool matchSerial)
{
  // State Lock : READY
  if (myState != FAUNA_STATE_READY)
  {
    return -0xFFFF;
  }

  // File open
  ifstream fin(pathFile);
  if (fin.is_open() == false)
  {
    return -1;
  }

  // Json parsing
  ordered_json json_in;
  try
  {
    fin >> json_in;
  }
  catch (const ordered_json::parse_error& e)
  {
    return -2;
  }
  catch (const std::exception& e) {
    return -3;
  }

  // Integrity Check
  if (_check_json_streamInfo(json_in) != 0)
  {
    return -0xFF;
  }

  // Congrats!
  return _read_json_streamInfo(json_in, matchSerial);
}





int fauna_do_launch_stream(bool fileStream)
{
  int ret = 0;

  // State Lock : READY
  if (myState != FAUNA_STATE_READY)
  {
    return -0xFFFF;
  }

  // Create DAQ taskset
  ret = _create_DAQTaskset();
  if (ret != 0)
  {
    _clear_DAQTaskset();
    return -0xFF00 + ret;
  }

  // File export
  if (fileStream == true)
  {
    // Directory creation
    fs::path path_local_stream = FAUNA_LOCAL + "stream/";
    fs::remove_all(path_local_stream);
    fs::create_directories(path_local_stream);

    // Export : streamInfo.json
    ret = fauna_write_json_streamInfo(path_local_stream.string().c_str(), "streamInfo");
    if (ret < 0)
    {
      return -0x0100 + ret;
    }

    // offsetInfo.json
    ret = _write_json_offsetInfo();
    if (ret < 0)
    {
      return -0x0100 + ret;
    }

    // bufferState.bin
    ret = _init_mmap_bin_bufferState(ret);
    if (ret < 0)
    {
      return -0x0100 + ret;
    }
  }

  // Bear a stream
  ret = _bear_stream(ret);
  if (ret != 0)
  {
    fauna_do_cease_stream();
  }
  else
  {
    // TRANSACTION : READY => RUNNING
    myState = FAUNA_STATE_RUNNING;
  }

  return ret;
}

int fauna_do_cease_stream()
{
  // State Lock : RUNNING
  if (myState != FAUNA_STATE_RUNNING)
  {
    return -0xFFFF;
  }

  // TRANSACTION : READY => RUNNING
  myState = FAUNA_STATE_READY;

  // kill the stream
  _kill_stream();

  // Clear DAQ taskset
  _clear_DAQTaskset();

  // Clear files
  fs::path path_local_stream = FAUNA_LOCAL + "stream/";
  fs::remove_all(path_local_stream);

  return 0;
}



int fauna_tell_buffer(std::string& nameDevice, int idxChannel, double* _buffer, int* _numSamplesRead)
{
  // State Lock : RUNNING
  if (myState != FAUNA_STATE_RUNNING)
  {
    return -0xFFFF;
  }

  // Error : No such device engaged
  if (listBufferInfo.find(nameDevice) == listBufferInfo.end())
  {
    return -1;
  }

  // Localize params
  int spb = listStreamParam[nameDevice].spb;
  int numChannel = listBufferInfo[nameDevice].numChannel;
  float64* src = buffer_daq[nameDevice][listBufferInfo[nameDevice].idxBuffer];  

  if (idxChannel == -1)
  {
    // Copy whole-device-buffer
    memcpy
    (
      _buffer, src, spb * numChannel * sizeof(double)
    );
    *_numSamplesRead = (int)listBufferInfo[nameDevice].numSampleReadPerChannel;
  }
  else if (idxChannel < 0)
  {
    // Error : No such magic word
    return -3;
  }
  else if (idxChannel < listBufferInfo[nameDevice].numChannel)
  {
    // Copy channel-specific-buffer
    memcpy
    (
      _buffer, src + spb * idxChannel, spb * sizeof(double)
    );
    *_numSamplesRead = (int)listBufferInfo[nameDevice].numSampleReadPerChannel;
  }
  else
  {
    // Error : Channel Idx out of range
    return -2;
  }

  return 0;
}

int fauna_tell_bufferInfo(std::string& nameDevice, double* _timeOut, bool* _idxBuffer, int* _numSamplesRead)
{
  // State Lock : RUNNING
  if (myState != FAUNA_STATE_RUNNING)
  {
    return -0xFFFF;
  }

  if (listBufferInfo.find(nameDevice) == listBufferInfo.end())
  {
    return -1;
  }

  *_timeOut = listBufferInfo[nameDevice].timeOut;
  *_idxBuffer = listBufferInfo[nameDevice].idxBuffer;
  *_numSamplesRead = listBufferInfo[nameDevice].numSampleReadPerChannel[listBufferInfo[nameDevice].idxBuffer];

  return 0;
}



// LOCAL

//==== DAQ TASK

int _create_DAQTaskset()
{
  // State Lock : READY
  if (myState != FAUNA_STATE_READY)
  {
    return -0xFFFF;
  }

  // Integrity Check 
  // MAYBE LATER (ASSUMED SAFE)

  // Task Creation
  TaskHandle* newTask;        
  int32 error;

  for (auto& dev : listStreamParam)
  {
    // Set Handle
    if (true)
    {
      listTaskHandle.insert({ dev.first, {} });
      newTask = &listTaskHandle[dev.first];
      error = DAQmxCreateTask("", newTask);
    }
    if (error != 0)
    {
      DAQmxClearTask(newTask);
      listTaskHandle.clear();
      return -1;
    }

    // Set Channels
    if (true)
    {
      string physicalChannel = _str_physicalChannel(listStreamChannel[dev.first]);
      error = DAQmxCreateAIVoltageChan(*newTask, physicalChannel.c_str(), "", DAQmx_Val_Cfg_Default, -dev.second.bias, dev.second.bias, DAQmx_Val_Volts, NULL);
    }
    if (error != 0)
    {
      DAQmxClearTask(newTask);
      listTaskHandle.clear();
      return -2;
    }

    // Set Buffer &Clock
    if (true)
    {
      DAQmxCfgInputBuffer(*newTask, dev.second.spb * 2);
      error = DAQmxCfgSampClkTiming(*newTask, NULL, dev.second.sps, DAQmx_Val_Rising, DAQmx_Val_ContSamps, dev.second.spb);
    }
    if (error != 0)
    {
      DAQmxClearTask(newTask);
      listTaskHandle.clear();
      return -3;
    }
  }

  return 0;
}

int _clear_DAQTaskset()
{
  if (myState != FAUNA_STATE_READY)
  {
    return -0xFFFF;
  }

  for (auto& task : listTaskHandle)
  {
    DAQmxClearTask(task.second);
  }
  listTaskHandle.clear();

  return 0;
}

//==== STREAM

int _bear_stream(int numMmapDev)
{
  // Malloc Buffer system
  _create_bufferSystem();

  vector<string> listDev;
  vector<STREAMPARAM> listParam;
  fauna_tell_listStreamDevice(&listDev, &listParam);

  int offset = 0;

  // Start Stream
  for (int i = 0; i < listDev.size(); i++)
  {
    // Start DAQ Task
    int ret = DAQmxStartTask(listTaskHandle[listDev[i]]);
    if (ret != 0)
    {
      return -1;
    }

    // Create Report Threads
    if (listParam[i].fileStream == true)
    {
      listThreadReport.emplace_back(_thread_report_buffer_mmap, listDev[i], numMmapDev, offset);
      offset++;
    }
    else
    {
      listThreadReport.emplace_back(_thread_report_buffer, listDev[i]);
    }
    
  }

  return 0;
}

int _kill_stream()
{
  // Stop Report Threads
  for (thread& th : listThreadReport)
  {
    th.join();
  }

  // Stop DAQ Tasks
  int ret = 0;
  for (auto& task : listTaskHandle)
  {
    if (DAQmxStopTask(task.second) != 0)
    {
      ret += 1;
    }
  }

  // Dalloc DAQ buffers
  _clear_bufferSystem();

  return ret;
}


int _create_bufferSystem()
{
  // Malloc buffer system
  __malloc_bufferSystem();

  // Create Buffer Info
  for (auto& dev : listStreamParam)
  {
    listBufferInfo.insert
    (
      {
        dev.first,
        {
          (int)(listStreamChannel[dev.first].size()),
          (float64)(dev.second.spb / dev.second.sps),
          true, // To start with 0
          {0, 0}
        }
      }
    );
  }

  return 0;
}

int _clear_bufferSystem()
{
  // Clear Buffer Info
  listBufferInfo.clear();

  // Dalloc BufferSystem
  __dalloc_bufferSystem();

  return 0;
}


int _read_json_streamInfo(ordered_json& json_in, bool matchSerial)
{
  // Make test deviceInfo
  const char* testDir = "./temp/";
  const char* testFileName = "testDeviceInfo";
  if (fauna_write_json_deviceInfo(testDir, testFileName) != 0)
  {
    return -0x0201;
  }

  // Load test file
  ifstream fin(string(testDir) + string(testFileName) + ".json");
  if (fin.is_open() == false)
  {
    return -0x0202;
  }

  // Test json parsing
  ordered_json json_test;
  try
  {
    fin >> json_test;
  }
  catch (const ordered_json::parse_error& e)
  {
    return -0x0203;
  }
  catch (const std::exception& e) {
    return -0x0204;
  }

  // Let the test begin

  // Version Check
  if (json_test["versionInfo"] != json_in["versionInfo"])
  {
    return -0x0205;
  }

  json_test = json_test["deviceInfo"]["listDevice"];
  for (auto& dev : json_in["deviceInfo"]["listDevice"].items())
  {
    if (json_test.contains(dev.key()) == false)
    {
      fauna_do_clear_streamDevice();
      return -0x0206;
    }

    if
    (
      (matchSerial == true)
      && (json_test[dev.key()]["serialCode"] != dev.value()["serialCode"])
    )
    {
      fauna_do_clear_streamDevice();
      return -0x0207;
    }
        
    if
    (
      (json_test[dev.key()]["minSps"] > dev.value()["sps"])
      || (json_test[dev.key()]["maxSps"] < dev.value()["sps"])
    )
    {
      fauna_do_clear_streamDevice();
      return -0x0208;
    }

    if
    (
      find(json_test[dev.key()]["listBias"].begin(), json_test[dev.key()]["listBias"].end(), dev.value()["bias"])
      == json_test[dev.key()]["listBias"].end()
    )
    {
      fauna_do_clear_streamDevice();
      return -0x0208;
    }
    
    if (json_test[dev.key()]["numChannel"] < dev.value()["numChannel"])
    {
      fauna_do_clear_streamDevice();
      return -0x0209;
    }

    vector<string> listChan = dev.value()["listChannel"].get<vector<string>>();
    for (auto& chan : listChan)
    {
      if
      (
        find(json_test[dev.key()]["listChannel"].begin(), json_test[dev.key()]["listChannel"].begin(), chan)
        == json_test[dev.key()]["listChannel"].end()
      )
      {
        fauna_do_clear_streamDevice();
        return -0x020A;
      }
    }

    // Insertion
    string nameDev = dev.key();
    STREAMPARAM param =
    {
      dev.value()["bias"],
      dev.value()["sps"],
      dev.value()["spb"],
      dev.value()["fileExport"]
    };

    int ret = fauna_do_insert_streamDevice(nameDev, param, listChan);
    if (ret < 0)
    {
      return -0x0300 + ret;
    }
  }

  return 0;
}



int _write_json_offsetInfo()
{
  ofstream fout(FAUNA_LOCAL + "stream/offsetInfo.json");

  if (fout.is_open() == false)
  {
    return -1;
  }

  ordered_json json_out;

  vector<string> listDev;
  vector<STREAMPARAM> listParam;
  fauna_tell_listStreamDevice(&listDev, &listParam);
  json_out["numDevice"] = listDev.size();

  // Count num of mmap devices
  int num_mmap_dev = 0;
  for (int i = 0; i < listDev.size(); i++)
  {
    if (listParam[i].fileStream == true)
    {
      num_mmap_dev++;
    }    
  }

  // Calc offsets
  int cnt_mmap_dev = 0;
  for (int i = 0; i < listDev.size(); i++)
  {
    if (listParam[i].fileStream == true)
    {
      ordered_json json_dev;
      json_dev["idxBuffer"] = cnt_mmap_dev;
      json_dev["numSamplesReadPerChannel"] = num_mmap_dev + cnt_mmap_dev;

      json_out["listDevice"][listDev[i]] = json_dev;
      cnt_mmap_dev++;
    }
  }

  fout << json_out.dump(2);

  return num_mmap_dev;
}

int _init_mmap_bin_bufferState(int numMmapDev)
{
  // file handle
  handle_file_bufferState = CreateFile
  (
    (FAUNA_LOCAL + "stream/bufferState.bin").data(),
    GENERIC_READ | GENERIC_WRITE,
    FILE_SHARE_READ,
    NULL,
    CREATE_ALWAYS,
    FILE_FLAG_RANDOM_ACCESS,
    NULL
  );

  if (handle_file_bufferState == 0)
  {
    return -1;
  }

  // mapping handle
  int64 sizeFile = (numMmapDev * 2) * (sizeof(int));
  int sizeFile_high = sizeFile >> 32;
  int sizeFile_low = sizeFile & 0xFFFFFFFF;

  handle_mapping_bufferState = CreateFileMapping
  (
    handle_file_bufferState,
    NULL,
    PAGE_READWRITE,
    sizeFile_high,
    sizeFile_low,
    "FAUNA_MMAP_BUFFERSTATE"
  );

  if (handle_mapping_bufferState == 0)
  {
    return -2;
  }
 
  // mmap viewer
  mmap_bufferState = (int*)MapViewOfFile
  (
    handle_mapping_bufferState,
    FILE_MAP_ALL_ACCESS,
    0,
    0,
    0
  );

  return 0;
}

//==== ==== BUFFER

int __malloc_bufferSystem()
{
  for (auto& dev : listStreamChannel)
  {
    if (dev.second.size() > 1)
    {
      buffer_daq.insert({ dev.first, {} });
      buffer_daq[dev.first] = new float64 * [2] {};

      buffer_daq[dev.first][0] = new float64[listStreamParam[dev.first].spb * dev.second.size()]{};
      buffer_daq[dev.first][1] = new float64[listStreamParam[dev.first].spb * dev.second.size()]{};
    }
  }
  return 0;
}

int __dalloc_bufferSystem()
{
  for (auto& dev : listStreamChannel)
  {
    delete[] buffer_daq[dev.first][0];
    delete[] buffer_daq[dev.first][1];
    delete[] buffer_daq[dev.first];
  }
  buffer_daq.clear();
  return 0;
}

int _init_mmap_bin_virtualBuffer(HANDLE* handle_file, HANDLE* handle_mapping, float64** viewer, int spb, int offsetDevice, int offsetChannel)
{
  // file handle
  *handle_file = CreateFile
  (
    (FAUNA_LOCAL + "stream/buffer/" + "virtualBuffer_" + to_string(offsetDevice) + "_" + to_string(offsetChannel) + ".bin").c_str(),
    GENERIC_READ | GENERIC_WRITE,
    FILE_SHARE_READ | FILE_SHARE_WRITE,
    NULL,
    CREATE_ALWAYS,
    FILE_FLAG_RANDOM_ACCESS,
    NULL
  );

  if (*handle_file == 0)
  {
    return -1;
  }

  // mapping handle
  int64 sizeFile = (spb * 2) * (sizeof(float64));
  int sizeFile_high = sizeFile >> 32;
  int sizeFile_low = sizeFile & 0xFFFFFFFF;
  string nameMap = "FAUNA_MMAP_VIRTUALBUFFER_" + to_string(offsetDevice) + "_" + to_string(offsetChannel);

  *handle_mapping = CreateFileMapping
  (
    *handle_file,
    NULL,
    PAGE_READWRITE,
    sizeFile_high,
    sizeFile_low,
    nameMap.c_str()
  );

  if (*handle_mapping == 0)
  {
    return -2;
  }

  // mmap viewer
  *viewer = (float64*)MapViewOfFile
  (
    *handle_mapping,
    FILE_MAP_ALL_ACCESS,
    0,
    0,
    0
  );

  return 0;
}

//==== Integrity

int _check_struct_streamParam(std::string& nameDevice, STREAMPARAM& streamParam)
{
  vector<double> listBias;
  fauna_tell_listBias(nameDevice, &listBias);

  if (find(listBias.begin(), listBias.end(), streamParam.bias) == listBias.end())
  {
    return -0x0101;
  }

  double minSps, maxSps;
  fauna_tell_rangeSps(nameDevice, &minSps, &maxSps);

  if (streamParam.sps < minSps || streamParam.sps > maxSps)
  {
    return -0x0102;
  }

  // SPB
  if (streamParam.spb <= 0)
  {
    return -0x0103;
  }

  return 0;
}

int _check_json_streamInfo(ordered_json& json_in)
{
  // Version
  string versionInfo = json_in["versionInfo"];
  if (versionInfo != VERSIONINFO)
  {
    return -0x0100;
  }

  // Number of devices
  if (json_in["deviceInfo"]["numDevice"] != json_in["deviceInfo"]["listDevice"].size())
  {
    return -0x0101;
  }

  // For each devices
  for (auto& dev : json_in["deviceInfo"]["listDevice"].items())
  {
    // Key Containment
    if
    (
      dev.value().contains("serialCode") == false
      || dev.value().contains("spb") == false
      || dev.value().contains("sps") == false
      || dev.value().contains("bias") == false
      || dev.value().contains("numChannel") == false
      || dev.value().contains("listChannel") == false
      || dev.value().contains("fileExport") == false
    )
    {
      return -0x0102;
    }

    // Type
    if
      (
        dev.value()["serialCode"].is_string() == false
        || dev.value()["spb"].is_number_integer() == false
        || dev.value()["spb"] <= 0
        || dev.value()["sps"].is_number_float() == false
        || dev.value()["sps"] <= 0
        || dev.value()["bias"].is_number_float() == false
        || dev.value()["bias"] <= 0
        || dev.value()["numChannel"].is_number_integer() == false
        || dev.value()["numChannel"] <= 0
        || dev.value()["listChannel"].is_array() == false
        || dev.value()["listChannel"].size() != dev.value()["numChannel"]
        || dev.value()["fileExport"].is_boolean() == false
        )
    {
      return -0x0103;
    }

    // List elements
    for (auto& chan : dev.value()["listChannel"])
    {
      if (chan.is_string() == false)
      {
        return -0x0104;
      }
    }
  } 

  return 0;
}

//==== THREAD

int _thread_report_buffer(std::string nameDevice)
{
  // Localize factors
  int spb = listStreamParam[nameDevice].spb;
  float64 timeOut = listBufferInfo[nameDevice].timeOut;
  int numChannel = listStreamChannel[nameDevice].size();
  
  bool* idxBuffer = &listBufferInfo[nameDevice].idxBuffer;
  TaskHandle* task = &listTaskHandle[nameDevice];
  float64** buffer = buffer_daq[nameDevice];
  int32* numSamplesReadPerChannel[2] = { &listBufferInfo[nameDevice].numSampleReadPerChannel[0], &listBufferInfo[nameDevice].numSampleReadPerChannel[1] };

  // Synchronize all threads
  while (myState == FAUNA_STATE_READY) {}

  // DAQ loop
  while (myState == FAUNA_STATE_RUNNING)
  {
    // BufferSwitch
    *idxBuffer = !(*idxBuffer);

    // READ
    _readAnalogF64(*task, spb, timeOut, numChannel, buffer[*idxBuffer], numSamplesReadPerChannel[*idxBuffer]);
  }

  return 0;
}

int _thread_report_buffer_mmap(std::string nameDevice, int numMmapDev, int offsetDevice)
{
  // Localize factors
  int spb = listStreamParam[nameDevice].spb;
  float64 timeOut = listBufferInfo[nameDevice].timeOut;
  int numChannel = listStreamChannel[nameDevice].size();

  bool* idxBuffer = &listBufferInfo[nameDevice].idxBuffer;
  TaskHandle* task = &listTaskHandle[nameDevice];
  float64** buffer = buffer_daq[nameDevice];
  int32* numSamplesReadPerChannel[2] = { &listBufferInfo[nameDevice].numSampleReadPerChannel[0], &listBufferInfo[nameDevice].numSampleReadPerChannel[1] };

  // Directory creation
  fs::path path_local_stream_buffer = FAUNA_LOCAL + "stream/buffer/";
  fs::create_directories(path_local_stream_buffer);

  // MMAP initilization
  HANDLE* list_handle_file = new HANDLE[numChannel]();
  HANDLE* list_handle_mapping = new HANDLE [numChannel]();
  float64** list_mmap_buffer = new float64* [numChannel]();

  for (int i = 0; i < numChannel; i++)
  { 
    _init_mmap_bin_virtualBuffer(&list_handle_file[i], &list_handle_mapping[i], &list_mmap_buffer[i], spb, offsetDevice, i);
  }  

  // Synchronize all threads
  while (myState == FAUNA_STATE_READY) {}

  // DAQ loop
  while (myState == FAUNA_STATE_RUNNING)
  {
    // BufferSwitch
    *idxBuffer = !(*idxBuffer);

    // READ
    _readAnalogF64(*task, spb, timeOut, numChannel, buffer[*idxBuffer], numSamplesReadPerChannel[*idxBuffer]);

    // MMAP : bufferState.bin
    *(mmap_bufferState + offsetDevice) = (int)(*idxBuffer);
    *(mmap_bufferState + numMmapDev + offsetDevice + 1) = (int)(*numSamplesReadPerChannel[*idxBuffer]);

    // MMAP : buffer instance
    for (int i = 0; i < numChannel; i++)
    {
      float64* dest = list_mmap_buffer[i] + (*idxBuffer == 0 ? 0 : spb);
      float64* src = buffer[*idxBuffer];

      memcpy(dest, src, (spb * sizeof(float64)));
    }
  }

  // dalloc
  delete[] list_handle_file;
  delete[] list_handle_mapping;
  delete[] list_mmap_buffer;

  return 0;
}


//==== inline

inline void _readAnalogF64(TaskHandle& taskHandle, int spb, float64 timeOut, int numChannel, float64* _buffer, int32* _numSamplesRead)
{
  DAQmxReadAnalogF64
  (
    taskHandle,
    spb,
    timeOut,
    DAQmx_Val_GroupByChannel,
    _buffer,
    spb * numChannel,
    _numSamplesRead,
    NULL
  );
}
