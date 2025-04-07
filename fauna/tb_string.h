#ifndef TRICKBAG_STRING
#define TRICKBAG_STRING

#include <string>
#include <vector>

int _tokenize(char*& fullString, std::string delimeter, std::vector<std::string>* _listToken);
std::string _str_physicalChannel(std::vector<std::string>& listChannel);


#endif