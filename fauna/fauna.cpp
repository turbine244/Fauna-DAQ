#include "fauna.h"

#include <set>
#include <algorithm>
#include <unordered_map>
#include <iostream>
#include <utility>
#include <thread>
#include <NIDAQmx.h>

#include "typedefFauna.h"
#include "tb_string.h"

using namespace std;

//====----====----====----====----====----====----====----====----VAL



//====----====----====----====----====----====----====----====----VAR

atomic<int> myState;
unordered_map<string, STREAMPARAM> listStreamParam;
unordered_map<string, vector<string>> listStreamChannel;

unordered_map<string, TaskHandle> listTaskHandle;
unordered_map<string, BUFFERINFO> listBufferInfo;
vector<thread> listThreadReport;

unordered_map<string, float64**> buffer_daq;

//====----====----====----====----====----====----====----====----FDEC

//==== DAQ TASK
int _create_DAQTaskset();
int _clear_DAQTaskset();

//==== STREAM
int _bear_stream();
int _kill_stream();

int _create_bufferSystem();
int _clear_bufferSystem();

//==== ==== BUFFER
int __malloc_bufferSystem();
int __dalloc_bufferSystem();

//==== THREAD
int _thread_report_buffer(std::string nameDevice);

//==== INLINE
inline void _readAnalogF64(TaskHandle& taskHandle, int spb, float64 timeOut, int numChannel, float64* _buffer, int32* _numSamplesRead);

//====----====----====----====----====----====----====----====----FDEF

// DLL_EXPORT

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

  // Return devices with analog inputs
  for (string dev : temp_dev)
  {
    vector<string> temp_chan;
    if (fauna_tell_listChannel(dev, &temp_chan) > 0)
    {
      _listDevice->push_back(dev);
    }
  }

  return 0;
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
  int ret = _tokenize(fullString, ", ", _listChannel);
  delete[] fullString;
  return ret;
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
  return 0;
}



int fauna_tell_listStreamDevice(std::vector<std::string>* _listDevice, std::vector<std::pair<double, int>>* _listParam)
{
  if (listStreamParam.empty())
  {
    return 1;
  }

  _listDevice->clear();
  if (_listParam != NULL)
  {
    _listParam->clear();
  }

  for (auto& dev : listStreamParam)
  {
    _listDevice->push_back(dev.first);

    if (_listParam != NULL)
    {
      _listParam->push_back({ dev.second.sps, dev.second.spb });
    }
  }

  return 0;
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
}



int fauna_do_insert_streamDevice(std::string& nameDevice, double customBias, double customSps, int customSpb, std::vector<std::string>& listChannel)
{
  // State Lock : READY
  if (myState != FAUNA_STATE_READY)
  {
    return -0xFFFF;
  }

  // Integrity Check
  if (customSps <= 0
    || customSpb <= 0
    || listChannel.empty()
    )
  {
    return -0xFF;
  }

  int ret = 0;

  // Param insertion
  if (listStreamParam.find(nameDevice) != listStreamParam.end())
  {
    ret += 0x0100;
  }
  listStreamParam.insert({ nameDevice, { customBias, customSps, customSpb } });

  // Channel insertion
  if (listStreamChannel.find(nameDevice) != listStreamChannel.end())
  {
    listStreamChannel[nameDevice].clear();
    ret += 0x0001;
  }
  listStreamChannel.insert({nameDevice, listChannel});
  
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



int fauna_do_launch_stream()
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

  // Bear a stream
  ret = _bear_stream();
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

int _bear_stream()
{
  // Malloc Buffer system
  _create_bufferSystem();

  // Start Stream
  for (auto& task : listTaskHandle)
  {
    // Start DAQ Task
    int ret = DAQmxStartTask(task.second);
    if (ret != 0)
    {
      return -1;
    }

    // Create Report Thread
    listThreadReport.emplace_back(_thread_report_buffer, task.first);
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
    for (string& chan : dev.second)
    {
      delete[] buffer_daq[dev.first][0];
      delete[] buffer_daq[dev.first][1];
      delete[] buffer_daq[dev.first];
    }
  }
  buffer_daq.clear();
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
  vector<string>* listChannel = &listStreamChannel[nameDevice];
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