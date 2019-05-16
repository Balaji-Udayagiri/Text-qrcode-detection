#include<bits/stdc++.h>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>

using namespace std;

int main()
{
	
	ofstream MyExcelFile("/home/debjoy/inventory_rev.csv");	
	MyExcelFile << "QR-code data, AlphaNumeric Code, Shelf Code" << endl;

	ifstream read("/home/debjoy/inventory.csv");
	string input, QR=" ", QRprev = " ", str[10][100], QR_list[10];
	int m=0, n=0;
	string line;
	while (std::getline(read, line)) // read whole line into line
	{	
		QRprev = QR;
		QR_list[n] = QR;
    	std::istringstream iss(line); // string stream
   		std::getline(iss, QR, ','); // read first part up to comma, ignore the comma
		if((QR!=QRprev)&&(m>1))
		{
			n++;
			m=0;
			QR_list[n-1]=QRprev;
			QR_list[n]=QR;
		}

    	iss >> str[n][m]; // read the second part
		m++;
	}
	read.close();
	
    //string str[3] = {"232J32KJ2K3J23J2H3K2H3K23K","21LK323J3J3KH23K4H23K","23K2J23K2J3K23KJ2K3K2"};
	for(int k=0; k<n+1; k++)
	{
	    int ma = -1;
	    map<string,int> mp;
	    for(int i=0;i<m;i++)
	    {
			if(str[k][i] == "") continue;
	        for(unsigned int j=0;j<str[k][i].length()-2;j++)
	        {
				//cout<<str[k][i]<<endl;
	            if(isdigit(str[k][i][j]) && isdigit(str[k][i][j+1]) && isalpha(str[k][i][j+2]))
	            {
	                string s = str[k][i].substr(j,3);
	                mp[s]++;
	                ma = max(ma,mp[s]);
	            }
	        }
	    }
	    for(map<string,int>::iterator itr = mp.begin(); itr!=mp.end(); itr++)
	    {
	        //cout<<itr->first<<" "<<itr->second<<endl;
	        if (itr->second == ma)
	        {
	            cout<<"Substring: "<<itr->first<<" count: "<<itr->second<<endl;
				string cc = QR_list[k]+","+itr->first+","+" ";
				MyExcelFile <<cc<< endl;
	        }
   	 	}
	
	
	
	}
	MyExcelFile.close();
}
