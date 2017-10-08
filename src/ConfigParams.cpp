#include <memory>
#include "ConfigParams.h"

using namespace std;
#ifdef USE_SINGLETON
ConfigParams * ConfigParams::pSelf= nullptr;


ConfigParams::~ConfigParams() {
	if (pSelf)
		delete pSelf;
}
#endif
