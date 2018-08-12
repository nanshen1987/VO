#ifndef CONFIG_H
#define CONFIG_H
#include"common_include.h"
namespace lmars{
  class Config
  {
  private:
      static shared_ptr<Config> config_;
      cv::FileStorage file_;
      Config(){};
  public:
      ~Config();
      static void setParameterFile(const string& fileName);
      template<typename T>
      static T get(const string& key)
      {
	return T(Config::config_->file_[key]);
      }
  };
}


#endif // CONFIG_H
