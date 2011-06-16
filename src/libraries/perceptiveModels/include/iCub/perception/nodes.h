/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
 * @note UNDER DEVELOPMENT   
 * @defgroup Nodes Nodes
 *  
 * @ingroup PerceptiveModels 
 *  
 * Abstract layers for dealing with perceptive models framework.
 *
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section intro_sec Description
 *
 * ... 
 *  
 */ 

#ifndef __PERCEPTIVEMODELS_NODES_H__
#define __PERCEPTIVEMODELS_NODES_H__

#include <map>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <iCub/perception/sensors.h>


namespace iCub
{

namespace perception
{

/**
* @ingroup Nodes
*
*/
class EventCallback
{
protected:
    std::string name;

public:
    EventCallback();
    std::string getName() const { return name; }
    virtual void execute() = 0;
};


/**
* @ingroup Nodes
*
*/
class Node
{
protected:
    std::string name;

    std::map<std::string,Sensor*>        sensors;
    std::map<std::string,EventCallback*> callbacks;
    std::map<std::string,Node*>          neighbors;

public:
    Node();
    std::string getName() const { return name; }

    void  attachSensor(Sensor &sensor);
    void  attachCallback(EventCallback &callback);
    void  addNeighbor(Node &node);
    bool  removeNeighbor(const std::string &name);
    Node* getNeighbor(const std::string &name) const;

    virtual bool fromProperty(const yarp::os::Property &options) = 0;
    virtual void toProperty(yarp::os::Property &options) const = 0;
    virtual bool calibrate(const yarp::os::Property &options) = 0;
    virtual bool getOutput(yarp::os::Value &out) const = 0;

    virtual ~Node() { }
};


}

}

#endif


