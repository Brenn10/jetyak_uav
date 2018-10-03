#ifndef VIRTUAL_RC_N3_H
#define VIRTUAL_RC_N3_H

// Include base class
#include "../../lib/dji_virtual_rc/include/virtual_rc.h"

class virtual_rc_N3 : public virtual_rc
{
public:
	using virtual_rc::virtual_rc;
	virtual ~virtual_rc_N3(){};

protected:
	// Callback functions
	void rcCallback(const sensor_msgs::Joy::ConstPtr& msg);
};
#endif
