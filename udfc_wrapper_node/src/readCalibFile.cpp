//#include "udfc_wrapper_node/udfc_wrapper_node.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

vector<double> readFromFile(){

vector<double> calibParams{}; //opens example calib file and returns a vector with calib params
    fstream newfile;
    newfile.open("1.txt",ios::in); //open a file to perform read operation using file object
   while(newfile.is_open()){ //checking whether the file is open
      string line;
      if(newfile.eof()){
        newfile.close();
        break;}

      newfile >> line;
      if((line != "[1080p]")&&(line.size()!=0)){
      line.erase(0,3);
      
      calibParams.push_back(stof(line));
      }

}
calibParams.pop_back();

return calibParams;

}



int main(){

vector<double> CalibrationVector = readFromFile();

for(int i = 0; i < CalibrationVector.size();++i){
     cout<<CalibrationVector[i]<<endl;

}

}



