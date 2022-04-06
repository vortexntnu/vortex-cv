//#include "udfc_wrapper_node/udfc_wrapper_node.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

int main(){
int i = 0;
vector<double> calibParams{};
//cout<<"Helllooo"<<endl;
    fstream newfile;
    newfile.open("1.txt",ios::in); //open a file to perform read operation using file object
   if (newfile.is_open()){ //checking whether the file is open
      string tp;
      while(getline(newfile, tp)){ //read data from file object and put it into string.
        i++;
         if(tp=="[1080p]"){
             cout<<"Jumping over [1080p]" <<endl;
         }
          else{
            tp.erase(0,3);
            //cout << tp << "\n"; //print the data of the string
            double a = stof(tp);
            if(a != 0){
                //cout << a <<endl;
                calibParams.push_back(stof(tp));}  
 
            }
            
            //cout<<1.3<<endl;
            //cout<<calibParams.size()<<endl;
           // cout<<mom<<endl;
                }
    
      newfile.close(); //close the file object. 
    for (int i = 0; i <= calibParams.size(); ++i)
    cout<<calibParams[i]<<endl;

}


}



