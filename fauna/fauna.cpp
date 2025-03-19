#include "fauna.h"

#include <set>
#include <unordered_map>
#include <iostream>
#include <utility>
#include <thread>
#include <NIDAQmx.h>

#include "enumFauna.h"
#include "typedefFauna.h"
#include "tb_string.h"

using namespace std;

//====----====----====----====----====----====----====----====----VAL

#define FAUNA_VOLTRANGE 5.0

//====----====----====----====----====----====----====----====----VAR

atomic<int> myState;
unordered_map<string, STREAMPARAM> listStreamParam;
unordered_map<string, vector<string>> listStreamChannel;

unordered_map<string, TaskHandle> listTaskHandle;
unordered_map<string, BUFFERINFO> listBufferInfo;
vector<thread> listThreadReport;

unordered_map<string, float64*> buffer_nominal;
unordered_map<string, unordered_map<string, float64**>> buffer_virtual;

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
int __malloc_buffer_nominal();
int __dalloc_buffer_nominal();

int __malloc_buffer_virtual();
int __dalloc_buffer_virtual();

//==== THREAD
int _thread_report_multiChannel(std::string nameDevice);
int _thread_report_singleChannel(std::string nameDevice);

//==== INLINE
inline void _readAnalogF64(TaskHandle& taskHandle, int spb, float64 timeOut, int numChannel, float64* _buffer, int32* _numSamplesRead);

//====----====----====----====----====----====----====----====----FDEF

// DLL_EXPORT

int fauna_tell_listDevice(std::vector<std::string>* _listDevice)
{
  // Get full string
  int32 bufferSize = DAQmxGetSysDevNames(nullptr, 0);
  char* fullString = new char[bufferSize + 1] {};
  DAQmxGetSysDevNames(fullString, bufferSize);

  // Tokenize
  vector<string> temp_dev = {};
  _tokenize(fullString, ", ", &temp_dev);

  // Return devices with analog inputs
  for (string dev : temp_dev)
  {
    vector<string> temp_chan;
    if (fauna_tell_listChannel(dev, &temp_chan) > 0)
    {
      _listDevice->push_back(dev);
    }
  }  

  delete[] fullString;
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

int fauna_tell_deviceSpsRange(std::string& nameDevice, double* _minSps, double* _maxSps)
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



int fauna_do_insert_streamDevice(std::string& nameDevice, double customSps, int customSpb, std::vector<std::string>& listChannel)
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

  listStreamParam.insert({ nameDevice, { customSps, customSpb } });
  
  if (listStreamChannel.find(nameDevice) == listStreamChannel.end())
  {
    listStreamChannel.insert({ nameDevice, listChannel });
    return 0;
  }
  else
  {
    for (auto& dev : listStreamChannel)
    {
      if (dev.first == nameDevice)
      {
        dev.second.assign(listChannel.begin(), listChannel.end());
        return 1;
      }
    }
  }
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
    return ret;
  }

  // TRANSACTION : READY => RUNNING
  myState = FAUNA_STATE_RUNNING;

  // Bear a stream
  ret = _bear_stream();
  if (ret != 0)
  {
    fauna_do_cease_stream();
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

int fauna_tell_buffer(std::string& nameDevice, std::string& nameChannel, double* _buffer)
{
  // State Lock : RUNNING
  if (myState != FAUNA_STATE_RUNNING)
  {
    return -0xFFFF;
  }

  if (buffer_virtual.find(nameDevice) == buffer_virtual.end())
  {
    return -1;
  }

  if (buffer_virtual[nameDevice].find(nameChannel) == buffer_virtual[nameDevice].end())
  {
    return -2;
  }

  memcpy
  (
    _buffer,
    buffer_virtual[nameDevice][nameChannel][listBufferInfo[nameDevice].idxBuffer],
    listBufferInfo[nameDevice].numSampleRead[listBufferInfo[nameDevice].idxBuffer * sizeof(double)]
  );
  return 0;
}

int fauna_tell_bufferInfo(std::string& nameDevice, double* _timeOut, bool* _idxBuffer, long* _numSamplesRead)
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
  *_numSamplesRead = listBufferInfo[nameDevice].numSampleRead[listBufferInfo[nameDevice].idxBuffer];

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
      error = DAQmxCreateAIVoltageChan(*newTask, physicalChannel.c_str(), "", DAQmx_Val_Cfg_Default, -FAUNA_VOLTRANGE, FAUNA_VOLTRANGE, DAQmx_Val_Volts, NULL);
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
    if (buffer_nominal.find(task.first) != buffer_nominal.end())
    {
      listThreadReport.emplace_back(_thread_report_multiChannel, task.first);
    }
    else
    {
      listThreadReport.emplace_back(_thread_report_singleChannel, task.first);
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
  // Malloc nominal buffer
  __malloc_buffer_nominal();

  // Malloc virtual buffers
  __malloc_buffer_virtual();

  // Create Buffer Info
  for (auto& dev : listStreamParam)
  {
    listBufferInfo.insert({ dev.first, {0, 0, {0, 0} } });
  }

  return 0;
}

int _clear_bufferSystem()
{
  // Clear Buffer Info
  listBufferInfo.clear();

  // Dalloc Virtual buffers
  __dalloc_buffer_virtual();

  // Dalloc Nominal buffers
  __dalloc_buffer_nominal();

  return 0;
}

//==== ==== BUFFER

int __malloc_buffer_nominal()
{
  for (auto& dev : listStreamChannel)
  {
    if (dev.second.size() > 1)
    {
      buffer_nominal.insert({ dev.first, new float64[dev.second.size() * listStreamParam[dev.first].spb] {} });
    }
  }
  return 0;
}

int __dalloc_buffer_nominal()
{
  for (auto& nom : buffer_nominal)
  {
    delete[] nom.second;
  }
  buffer_nominal.clear();

  return 0;
}


int __malloc_buffer_virtual()
{
  for (auto& dev : listStreamChannel)
  {
    buffer_virtual.insert({ dev.first, {} });

    for (string& chan : dev.second)
    {
      buffer_virtual[dev.first].insert({ chan, new float64* [2] {} });
      buffer_virtual[dev.first][chan][0] = new float64[listStreamParam[dev.first].spb] {};
      buffer_virtual[dev.first][chan][1] = new float64[listStreamParam[dev.first].spb] {};
    }
  }
  return 0;
}

int __dalloc_buffer_virtual()
{
  for (auto& dev : listStreamChannel)
  {
    for (string& chan : dev.second)
    {
      delete[] buffer_virtual[dev.first][chan][0];
      delete[] buffer_virtual[dev.first][chan][1];
      delete[] buffer_virtual[dev.first][chan];
    }
  }
  buffer_virtual.clear();

  return 0;
}


//==== THREAD

int _thread_report_singleChannel(std::string nameDevice)
{
  string* nameChannel = &listStreamChannel[nameDevice][0];

  TaskHandle* task = &listTaskHandle[nameDevice];
  int spb = listStreamParam[nameDevice].spb;
  int numChannel = listStreamChannel[nameDevice].size();
  float64** buffer_v = buffer_virtual[nameDevice][*nameChannel];
  bool* idxBuffer = &listBufferInfo[nameDevice].idxBuffer;

  listBufferInfo[nameDevice].timeOut = spb / listStreamParam[nameDevice].sps;
  *idxBuffer = 0;

  while (myState == FAUNA_STATE_RUNNING)
  {
    // BufferSwitch
    *idxBuffer = !(*idxBuffer);

    // READ
    _readAnalogF64(*task, spb, listBufferInfo[nameDevice].timeOut, numChannel, buffer_v[*idxBuffer], &listBufferInfo[nameDevice].numSampleRead[listBufferInfo[nameDevice].idxBuffer]);
  }

  return 0;
}

int _thread_report_multiChannel(std::string nameDevice)
{
  TaskHandle* task = &listTaskHandle[nameDevice];
  int spb = listStreamParam[nameDevice].spb;
  int numChannel = listStreamChannel[nameDevice].size();
  float64* buffer_n = buffer_nominal[nameDevice];
  bool* idxBuffer = &listBufferInfo[nameDevice].idxBuffer;

  listBufferInfo[nameDevice].timeOut = spb / listStreamParam[nameDevice].sps;
  *idxBuffer = 0;

  // Buffer virtualization
  float64*** buffer_v = new float64**[numChannel] {};
  for (int i = 0; i < numChannel; i++)
  {
    buffer_v[i] = new float64* [2] {};
    buffer_v[i][0] = buffer_virtual[nameDevice][listStreamChannel[nameDevice][i]][0];
    buffer_v[i][1] = buffer_virtual[nameDevice][listStreamChannel[nameDevice][i]][1];
  }

  while (myState == FAUNA_STATE_RUNNING)
  {
    // BufferSwitch
    *idxBuffer = !(*idxBuffer);

    // READ
    _readAnalogF64(*task, spb, listBufferInfo[nameDevice].timeOut, numChannel, buffer_n, &listBufferInfo[nameDevice].numSampleRead[listBufferInfo[nameDevice].idxBuffer]);

    // Split Channels
    for (int i = 0; i < numChannel; i++)
    {
      memcpy(
        buffer_v[i][*idxBuffer],
        buffer_n + (i * spb),
        spb * sizeof(float64)
      );
    }
  }

  return 0;
}


//==== inline

inline void _readAnalogF64 (TaskHandle& taskHandle, int spb, float64 timeOut, int numChannel, float64* _buffer, int32* _numSamplesRead)
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