#include "lmars/config.h"
namespace lmars{
void Config::setParameterFile(const string& fileName)
{
  if(config_==nullptr)
    config_=shared_ptr<Config>(new Config);
  config_->file_=cv::FileStorage(fileName.c_str(),cv::FileStorage::READ);
  if(config_->file_.isOpened()==false){
    cerr<<"param file "<<fileName<<" does not exist."<<endl;
    config_->file_.release();
    return;
  }
}
Config::~Config()
{
  if(file_.isOpened())
    file_.release();
}


shared_ptr<Config> Config::config_=nullptr;
  
}
