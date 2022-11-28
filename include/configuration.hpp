#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <string>

struct Configuration
{
	Configuration(const std::string& configFileFullPath);

	bool writeProperty(const std::string& property, const std::string& value);

	void print();

	float rGate, alpha, beta, gamma, rMax, lMin, lMax, costMin; 
	unsigned int lookBack, lookBackSpecific, nConfirm, nConfirmSpecifically;
	bool importDetections;

	bool good;
	const unsigned int nProperties;
};

#endif