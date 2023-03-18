#include "autocomplete.h"


void AutoComplete::add_match_string(std::string &s) { strs_.emplace_back(s); }

std::string AutoComplete::last_string() const { return last_str_; }

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
