#ifndef UNHUMAN_MOTORLIB_AUTOCOMPLETE_H_
#define UNHUMAN_MOTORLIB_AUTOCOMPLETE_H_

#include <algorithm>
#include <string>
#include <vector>

class AutoComplete {
 public:
  void add_match_string(std::string &s);
  std::string autocomplete(char c);
  std::string last_string() const;

 private:
  char last_key_ = 0;
  std::string str_, last_str_;
  std::vector<std::string> strs_;
};

// Return longest common string starting at the beginning.
inline std::string max_string_match(std::string &s1, std::string s2) {
  // TODO: See if it is possible to replace with the STL.
  unsigned int i;
  for (i = 0; i < std::min(s1.size(), s2.size()); i++) {
    if (s1[i] != s2[i]) {
      break;
    }
  }
  return s1.substr(0, i);
}


#endif  // UNHUMAN_MOTORLIB_AUTOCOMPLETE_H_
