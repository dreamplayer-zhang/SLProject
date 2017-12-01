//#############################################################################
//  File:      SLCVKeyframeDB.h
//  Author:    Michael G�ttlicher
//  Date:      October 2017
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/Coding-Style-Guidelines
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#ifndef SLCVKEYFRAMEDB_H
#define SLCVKEYFRAMEDB_H

#include <SLCamera.h>
#include <SLCVKeyFrame.h>

//-----------------------------------------------------------------------------
//! AR Keyframe database class
/*! 
*/
class SLCVKeyFrameDB
{
public:

    SLCVVKeyFrame& keyFrames() { return _keyFrames; }

protected:


private:
    SLCVVKeyFrame _keyFrames;
};

#endif // !SLCVKEYFRAMEDB_H