#ifndef UUID_HPP
#define UUID_HPP

#include <vector>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>

namespace uwds {

  #define NEW_UUID Uuid().toString();

  class Uuid
  {
  public:
    Uuid() {
      uuid_ = boost::uuids::to_string(boost::uuids::random_generator()());
    }
    std::string toString() {
      std::vector<std::string> tokens;
      boost::algorithm::split(tokens, uuid_, boost::is_any_of("-"));
      return boost::algorithm::join(tokens, "");
    }
    std::string uuid_;
  };
}

#endif
