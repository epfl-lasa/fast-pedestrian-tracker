#include "configuration.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>

using namespace std;

Configuration::	Configuration(const string& configFileFullPath)
: nProperties(13)
{
	good = false;

	unsigned int count = 0;

	ifstream configFile(configFileFullPath);

	if (!configFile.good())
		return;

	for (string line; getline(configFile, line); ) 
	{
		istringstream iss(line);
		vector<string> tokens;
		copy(istream_iterator<string>(iss),
			 istream_iterator<string>(),
			 back_inserter(tokens));

		string property, value;
		unsigned int i = 0;
		for (auto& word : tokens)
		{
			if (word[0] == '#')
				break;
			if (i == 0)
			{
				property = word;
				i++;
			}
			else if (i == 1)
			{
				value = word;
				i++;
				break;
			}
		}
		if (i == 2)
		{
			if (writeProperty(property, value))
				count++;
		}
	}

	if (count == nProperties)
		good = true;
}

bool Configuration::writeProperty(const string& property, const string& value)
{
	if (property == "rGate")
	{
		rGate = stod(value);
		return true;
	}
	else if (property == "lookBack")
	{
		lookBack = stoi(value);
		return true;
	}
	else if (property == "lookBackSpecific")
	{
		lookBackSpecific = stoi(value);
		return true;
	}
	else if (property == "nConfirm")
	{
		nConfirm = stoi(value);
		return true;
	}
	else if (property == "importDetections")
	{
		importDetections = (stoi(value) != 0);
		return true;
	}
	else if (property == "nConfirmSpecifically")
	{
		nConfirmSpecifically = stoi(value);
		return true;
	}
	else if (property == "alpha")
	{
		alpha = stod(value);
		return true;
	}
	else if (property == "beta")
	{
		beta = stod(value);
		return true;
	}
	else if (property == "gamma")
	{
		gamma = stod(value);
		return true;
	}
	else if (property == "rMax")
	{
		rMax = stod(value);
		return true;
	}
	else if (property == "lMin")
	{
		lMin = stod(value);
		return true;
	}
	else if (property == "lMax")
	{
		lMax = stod(value);
		return true;
	}
	else if (property == "costMin")
	{
		costMin = stod(value);
		return true;
	}
	return false;
}

void Configuration::print()
{
	cout << "rGate " << rGate << std::endl;
	cout << "lookBack " << lookBack << std::endl;
	cout << "nConfirm " << nConfirm << std::endl;
	cout << "importDetections " << importDetections << std::endl;
	cout << "nConfirmSpecifically " << nConfirmSpecifically << std::endl;
	cout << "alpha " << alpha << std::endl;
	cout << "beta " << beta << std::endl;
	cout << "rMax " << rMax << std::endl;
	cout << "lMin " << lMin << std::endl;
	cout << "lMax " << lMax << std::endl;
	cout << "costMin " << costMin << std::endl;
}

/*
int main()
{
	Configuration c("/home/gonond/github/mot/fast-pedestrian-tracker/config.txt");
	cout << c.rGate << std::endl;
	cout << c.lookBack << std::endl;
	cout << c.nConfirm << std::endl;
	cout << c.alpha << std::endl;
	cout << c.beta << std::endl;
	cout << c.rMax << std::endl;
	cout << c.lMin << std::endl;
	cout << c.lMax << std::endl;
	cout << c.costMin << std::endl;
}
*/