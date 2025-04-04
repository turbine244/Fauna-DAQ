#ifndef DLL_FAUNA
#define DLL_FAUNA

#ifdef FAUNA_EXPORTS
#define FAUNA_DECLSPEC __declspec(dllexport)
#else
#define FAUNA_DECLSPEC __declspec(dllimport)
#endif

#include <vector>
#include <string>

enum
{
  FAUNA_AUTO = 0,

  FAUNA_STATE_READY = 0,
  FAUNA_STATE_RUNNING,
};

// DLL EXPORT

// -1
extern "C" FAUNA_DECLSPEC int fauna_tell_state(int* _state);

// 0
extern "C" FAUNA_DECLSPEC int fauna_tell_listDevice(std::vector<std::string>* _listDevice);
extern "C" FAUNA_DECLSPEC int fauna_tell_listChannel(std::string& nameDevice, std::vector<std::string>* _listChannel);
extern "C" FAUNA_DECLSPEC int fauna_tell_rangeSps(std::string& nameDevice, double* _minSps, double* _maxSps);
extern "C" FAUNA_DECLSPEC int fauna_tell_listBias(std::string& nameDevice, std::vector<double>* _listBias);

// 1
extern "C" FAUNA_DECLSPEC int fauna_tell_listStreamDevice(std::vector<std::string>* _listDevice, std::vector<std::pair<double, int>>* _listParam);
extern "C" FAUNA_DECLSPEC int fauna_tell_listStreamChannel(std::string& nameDevice, std::vector<std::string>* _listChannel);
extern "C" FAUNA_DECLSPEC int fauna_do_insert_streamDevice(std::string& nameDevice, double customSps, int customSpb, std::vector<std::string>& listStreamChannel);
extern "C" FAUNA_DECLSPEC int fauna_do_erase_streamDevice(std::string& nameDevice);
extern "C" FAUNA_DECLSPEC int fauna_do_clear_streamDevice();
extern "C" FAUNA_DECLSPEC int fauna_do_launch_stream();

// 2
extern "C" FAUNA_DECLSPEC int fauna_do_cease_stream();
extern "C" FAUNA_DECLSPEC int fauna_tell_buffer(std::string& nameDevice, int idxChannel, double* _buffer, int* _numSamplesRead);
extern "C" FAUNA_DECLSPEC int fauna_tell_bufferInfo(std::string& nameDevice, double* _timeOut, bool* _idxBuffer, int* _numSamplesRead);

#endif