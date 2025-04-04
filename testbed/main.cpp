#include <iostream>
#include <fstream>
#include <Windows.h>

#include "../fauna/fauna.h"
#pragma comment(lib, "../x64/Debug/fauna.lib")

#define SPS 48000
#define SPB 48000

using namespace std;

int scenario1(); // Mere Showcase

int main()
{
  return scenario1();
}

int scenario1()
{
  int ret = 0;

  // fauna_tell_listDevice
  cout << "!! List of Devices (with Analog Input) : " << endl;

  vector<string> listDevice = {};
  fauna_tell_listDevice(&listDevice);

  if (listDevice.size() == 0)
  {
    cout << "No device? Pathetic." << endl;
    return -1;
  }

  for (string dev : listDevice)
  {
    cout << dev << endl;
  }
  cout << endl;

  // fauna_tell_listChannel
  cout << "!! List of Analog Input Channels : " << endl;

  for (string dev : listDevice)
  {
    vector<string> listChannel = {};
    fauna_tell_listChannel(dev, &listChannel);

    cout << dev << "(has " << listChannel.size() << ")" << endl;
    for (string chan : listChannel)
    {
      cout << "い" << chan << endl;
    }
  }
  cout << endl;

  // fauna_tell_rangeSps & fauna_tell_listBias
  cout << "!! Device Specs : " << endl;
  for (string dev : listDevice)
  {
    cout << dev << endl;
    
    double minSps{ 0 }, maxSps{ 0 };
    fauna_tell_rangeSps(dev, &minSps, &minSps);
    
    cout << "いSPS Range : from " << minSps << " Hz to " << maxSps << " Hz" << endl;

    vector<double> listBias;
    fauna_tell_listBias(dev, &listBias);

    cout << "いBIAS Candidates(got " << listBias.size() << ")" << endl;
    for (double bias : listBias)
    {
      cout << " い" << bias << "V" << endl;
    }
  }

  cout << endl;

  // fauna_do_insert_streamDevice
  cout << "!! Device-Stream engagemnet : " << endl;
  for (string dev : listDevice)
  {
    vector<string> listChannel = {};
    fauna_tell_listChannel(dev, &listChannel);

    int ret = fauna_do_insert_streamDevice(dev, 48000, 48000, listChannel);
    if (ret == 0)
    {
      cout << dev << " engaged successfully." << endl;
    }
    else
    {
      cout << dev << " engage failed.." << endl;
      return ret;
    }
  }
  cout << endl;

  // fauna_tell_listStreamDevice & fauna_tell_listStreamChannel
  cout << "!! List of Stream engaged Devices : " << endl;
  if (true)
  {
    vector<string> listStreamDevice = {};
    vector<pair<double, int>> listStreamParam = {};

    fauna_tell_listStreamDevice(&listStreamDevice, &listStreamParam);

    for (int i = 0; i < listStreamDevice.size(); i++)
    {
      cout << listStreamDevice[i] << endl;
      cout << "いSPS : " << listStreamParam[i].first << " Hz" << endl;
      cout << "いSPB : " << listStreamParam[i].second << " Samples" << endl;
      cout << "いTimal Length : " << (double)listStreamParam[i].second / listStreamParam[i].first << " sec" << endl;

      vector<string> listStreamChannel = {};
      fauna_tell_listStreamChannel(listStreamDevice[i], &listStreamChannel);

      cout << "いChannels engaged(" << listStreamChannel.size() << ")" << endl;

      for (int j = 0; j < listStreamChannel.size(); j++)
      {
        cout << " い" << listStreamChannel[j] << endl;
      }
      cout << endl;
    }
  }

  // fauna_do_erase_streamDevice
  // clear? nope..
  cout << "!! Device-Stream dis-engagement : " << endl;
  cout << "Test erase : " << listDevice[0] << "\n..." << endl;

  ret = fauna_do_erase_streamDevice(listDevice[0]);
  if (ret == 0)
  {
    cout << listDevice[0] << " dis-engaged successfully. (Gonna re-engage soon don't worry)" << endl;
    cout << "..." << endl;

    // Re-engage
    vector<string> listChannel = {};
    fauna_tell_listChannel(listDevice[0], &listChannel);

    int ret = fauna_do_insert_streamDevice(listDevice[0], SPS, SPB, listChannel);
    if (ret == 0)
    {
      cout << listDevice[0] << " re-engaged successfully." << endl;
    }
    else
    {
      cout << listDevice[0] << " re-engage failed.." << endl;
      return ret;
    }
  }
  else
  {
    cout << listDevice[0] << " dis-engage failed.." << endl;
    return ret;
  }
  cout << endl;

  // fauna_do_launch_stream
  cout << "!! Launching Stream" << endl;
  ret = fauna_do_launch_stream();
  if (ret == 0)
  {
    cout << "Stream Launched Successfully!" << endl;
  }
  else
  {
    cout << "Stream Launch Failed.." << endl;
    return ret;
  }
  cout << endl;

  // fauna_tell_bufferInfo & fauna_tell_state & fauna_tell_buffer

  cout << "!! Streaming for 10secs" << endl;
  double* retailBuffer = new double[SPB] {};
  double timeOut = 0;
  bool idxBuffer = 0;
  int numSamplesRead = 0;
  string nameChannel = "";

  if (true)
  {
    vector<string> listChannel = {};
    fauna_tell_listChannel(listDevice[0], &listChannel);
    fauna_tell_bufferInfo(listDevice[0], &timeOut, &idxBuffer, &numSamplesRead);
    nameChannel = listChannel[0];

    cout << nameChannel << "'s latest buffer[0] is printed : " << endl;

    ofstream fileOut("dataAcquired.bin", ios::binary | ios::out);

    int stateCode = -1;

    while (stateCode != FAUNA_STATE_RUNNING)
    {
      cout << "Synchronizing task threads.." << endl;
      fauna_tell_state(&stateCode);
    }

    int secCnt = 1;
    while (secCnt--)
    {
      fauna_tell_buffer(listDevice[0], 0, retailBuffer, &numSamplesRead);
      cout << retailBuffer[0] << endl;

      fileOut.write(reinterpret_cast<const char*>(retailBuffer), sizeof(double) * SPB);

      Sleep(1000);
    }

    cout << "File exported : dataAcquired.bin" << endl;
  }
  cout << endl;

  // fauna_do_cease_stream

  cout << "!! Disposing Stream" << endl;
  if (true)
  {
    ret = fauna_do_cease_stream();

    if (ret)
    {
      cout << "Stream ceased Successfully!" << endl;
    }
    else
    {
      cout << "Stream cessation Failed.." << endl;
      return ret;
    }
  }
  cout << endl;

  return 0;
}