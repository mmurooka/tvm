/* Author: Masaki Murooka */

#ifndef HRP5P_SETTING_
#define HRP5P_SETTING_

const std::map<std::string, std::vector<double>> hrp5p_initial_q = {
  {"Root", {1.0, 0.0, 0.0, 0.0, -0.022477, 0.0, 0.790427}},
  {"RCY", {0.0}},
  {"RCR", {0.0}},
  {"RCP", {-0.46897}},
  {"RKP", {0.872665}},
  {"RAP", {-0.403695}},
  {"RAR", {0.0}},
  {"LCY", {0.0}},
  {"LCR", {0.0}},
  {"LCP", {-0.46897}},
  {"LKP", {0.872665}},
  {"LAP", {-0.403695}},
  {"LAR", {0.0}},
  {"WP", {0.0}},
  {"WR", {0.0}},
  {"WY", {0.0}},
  {"HY", {0.0}},
  {"HP", {0.0}},
  {"RSC", {0.0}},
  {"RSP", {0.523599}},
  {"RSR", {-0.349066}},
  {"RSY", {0.0}},
  {"REP", {-1.309}},
  {"RWRY", {0.0}},
  {"RWRR", {-0.523599}},
  {"RWRP", {0.0}},
  {"RHDY", {0.0}},
  {"RTMP", {0.05236}},
  {"RTPIP", {0.0}},
  {"RTDIP", {0.0}},
  {"RIMP", {-0.05236}},
  {"RIPIP", {0.0}},
  {"RIDIP", {0.0}},
  {"RMMP", {-0.05236}},
  {"RMPIP", {0.0}},
  {"RMDIP", {0.0}},
  {"LSC", {0.0}},
  {"LSP", {0.523599}},
  {"LSR", {0.349066}},
  {"LSY", {0.0}},
  {"LEP", {-1.309}},
  {"LWRY", {0.0}},
  {"LWRR", {0.523599}},
  {"LWRP", {0.0}},
  {"LHDY", {0.0}},
  {"LTMP", {-0.05236}},
  {"LTPIP", {0.0}},
  {"LTDIP", {0.0}},
  {"LIMP", {0.05236}},
  {"LIPIP", {0.0}},
  {"LIDIP", {0.0}},
  {"LMMP", {0.05236}},
  {"LMPIP", {0.0}},
  {"LMDIP", {0.0}}
};

const std::vector<std::string> hrp5p_filtered_link_names = {
  "Rthumb_Link0", "Rthumb_Link1", "Rthumb_Link2",
  "Rindex_Link0", "Rindex_Link1", "Rindex_Link2",
  "Rmiddle_Link0", "Rmiddle_Link1", "Rmiddle_Link2",
  "Lthumb_Link0", "Lthumb_Link1", "Lthumb_Link2",
  "Lindex_Link0", "Lindex_Link1", "Lindex_Link2",
  "Lmiddle_Link0", "Lmiddle_Link1", "Lmiddle_Link2"
};

#endif
