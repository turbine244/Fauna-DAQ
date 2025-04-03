#include "tb_string.h"

using namespace std;

int _tokenize(char*& fullString, std::string delimeter, std::vector<std::string>* _listToken)
{
  char* context = nullptr;
  char* token = strtok_s(fullString, delimeter.c_str(), &context);

  long long cnt = 0;
  while (true) {
    if (token == nullptr)
    {
      break;
    }
    else
    {
      _listToken->push_back(token);
      cnt++;
    }
    token = strtok_s(nullptr, delimeter.c_str(), &context);
  }

  return cnt;
}

std::string _str_physicalChannel(std::vector<std::string>& listChannel)
{
  string ret = "";
  for (string ch : listChannel)
  {
    ret += ch;
    ret += ", ";
  }
  ret.pop_back();
  ret.pop_back();

  return ret;
}