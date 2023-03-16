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

// === Implementation ===
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
void AutoComplete::add_match_string(std::string &s) { strs_.emplace_back(s); }
std::string AutoComplete::autocomplete(char c) {
  std::vector<std::string> matches;
  std::string str_out;
  switch (c) {
    case '\t': {
      auto &str = str_;
      std::copy_if(strs_.begin(), strs_.end(), std::back_inserter(matches),
                   [&str](std::string s) { return s.rfind(str, 0) == 0; });
      if (matches.size() == 1) {
        str_ = matches[0];
        str_out = '\r' + str_;
      } else if (matches.size()) {
        std::string max_match = matches[0];
        if (last_key_ == '\t') {
          str_out = '\n';
          std::string max_match = matches[0];
          for (auto &match : matches) {
            max_match = max_string_match(max_match, match);
            str_out += match + '\t';
          }
          // Move str_ out to farthest in common characters.
          str_ = max_match;
          str_out += '\n' + str_;
        } else {
          for (auto &match : matches) {
            max_match = max_string_match(max_match, match);
          }
          str_ = max_match;
          str_out = '\r' + str_;
        }
      }
      break;
    }
    case '\n':
      str_out = '\n';
      if (str_ != "") {
        last_str_ = str_;
      }
      str_ = "";
      break;
    case 127:
      if (str_.size()) {
        str_.erase(str_.end() - 1, str_.end());
        str_out = "\b \b";
      }
      break;
    default:
      str_ += c;
      str_out = c;
      break;
  }
  last_key_ = c;
  return str_out;
}
std::string AutoComplete::last_string() const { return last_str_; }

#endif  // UNHUMAN_MOTORLIB_AUTOCOMPLETE_H_
