//#############################################################################
//  File:      SLGLVertexBuffer.h
//  Purpose:   Wrapper class around OpenGL Vertex Buffer Objects (VBO) 
//  Author:    Marcus Hudritsch
//  Date:      January 2016
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/Coding-Style-Guidelines
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#ifndef SLGLVERTEXBUFFER_H
#define SLGLVERTEXBUFFER_H

#include <stdafx.h>
#include <SLGLEnums.h>

//-----------------------------------------------------------------------------
//! Struct for vertex attribute information
struct SLGLAttribute
{   SLGLAttributeType   type;           //!< type of vertex attribute
    SLint               elementSize;    //!< size of attribute element (SLVec3f has 3)
    SLuint              offsetBytes;    //!< offset of the attribute data in the buffer
    SLuint              bufferSizeBytes;//!< size of the attribute part in the buffer
    void*               dataPointer;    //!< pointer to the attributes source data
    SLint               location;       //!< GLSL input variable location index
    SLbool              convertToHalf;  //!< Flag if float attribute is converted to half float 
};
//-----------------------------------------------------------------------------
typedef vector<SLGLAttribute>  SLVVertexAttrib;
//-----------------------------------------------------------------------------




//-----------------------------------------------------------------------------
//! SLGLVertexBuffer encapsulates an OpenGL vertex buffer object
/*!
*/
class SLGLVertexBuffer
{
    public:
                    SLGLVertexBuffer    ();
                   ~SLGLVertexBuffer    () {clear(BT_float);}
        
        //! Sets the buffer data type (float or half float)
        void        dataType            (SLGLBufferType bt);

        //! Deletes all vertex array & vertex buffer objects
        void        deleteGL            ();

        //! Clears the attribute definition and sets the buffer data type
        void        clear               (SLGLBufferType bt);

        //! Returns the vector index if a vertex attribute exists otherwise -1
        SLint       attribIndex         (SLGLAttributeType type);

                
        //! Updates a specific vertex attribute in the VBO
        void        updateAttrib        (SLGLAttributeType type, 
                                         SLint elementSize, 
                                         void* dataPointer);
        
        //! Updates a specific vertex attribute in the VBO
        void        updateAttrib        (SLGLAttributeType type, 
                                         SLVfloat& data) {updateAttrib(type, 1, (void*)&data[0]);}
        
        //! Updates a specific vertex attribute in the VBO
        void        updateAttrib        (SLGLAttributeType type, 
                                         SLVVec2f& data) {updateAttrib(type, 2, (void*)&data[0]);}

        //! Updates a specific vertex attribute in the VBO
        void        updateAttrib        (SLGLAttributeType type, 
                                         SLVVec3f& data) {updateAttrib(type, 3, (void*)&data[0]);}

        //! Updates a specific vertex attribute in the VBO
        void        updateAttrib        (SLGLAttributeType type, 
                                         SLVVec4f& data) {updateAttrib(type, 4, (void*)&data[0]);}

        //! Generates the VBO
        void        generate            (SLuint numVertices, 
                                         SLGLBufferUsage usage = BU_static,
                                         SLbool outputInterleaved = true);
        
        //! Binds & enables the vertex attribute
        void        bindAndEnableAttrib ();
        
        //! disables the vertex attribute
        void        disableAttrib ();

        // Getters
        SLint       id                  () {return _id;}
        SLVVertexAttrib& attribs        () {return _attribs;} 
        SLbool      outputInterleaved   () {return _outputInterleaved;}

        // Some statistics
        static SLuint totalBufferCount;     //! static total no. of buffers in use
        static SLuint totalBufferSize;      //! static total size of all buffers in bytes
        
        //! Returns the size of a buffer data type
        static SLint sizeOfType(SLGLBufferType type);
                                               
    protected:
        SLuint          _id;                //! OpenGL id of vertex buffer object
        SLuint          _numVertices;       //! NO. of vertices in array
        SLGLBufferType  _dataType;          //! Data Type              
        SLVVertexAttrib _attribs;           //! Vector of vertex attributes 
        SLbool          _outputInterleaved; //! Flag if VBO should be generated interleaved
        SLint           _strideBytes;       //! Distance for interleaved attributes in bytes
        SLuint          _sizeBytes;         //! Total size of float VBO in bytes
        SLGLBufferUsage _usage;             //! buffer usage (static, dynamic or stream)
};
//-----------------------------------------------------------------------------

#endif
