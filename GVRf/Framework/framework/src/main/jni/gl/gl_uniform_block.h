
/* Copyright 2015 Samsung Electronics Co., LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef GL_UNIFORMBLOCK_H_
#define GL_UNIFORMBLOCK_H_

#include "objects/uniform_block.h"

namespace gvr
{

/**
 * Manages a Uniform Block containing data parameters to pass to
 * OpenGL vertex and fragment shaders.
 *
 * The UniformBlock may be updated by the application. If it has changed,
 * GearVRf resends the entire data block to OpenGL.
 */
    class GLUniformBlock : public UniformBlock
    {
    public:
        explicit GLUniformBlock(const char *descriptor, int bindingPoint, const char *blockName);
        explicit GLUniformBlock(const char *descriptor, int bindingPoint, const char *blockName, int maxelems);

        virtual ~GLUniformBlock();

        /**
         * Copy the data from CPU into the OpenGL uniform buffer.
         * @param Renderer  Renderer to use
         * @param start     starting byte to copy to GPU
         * @param len       number of bytes to copy to GPU
         */
        virtual bool updateGPU(Renderer *, int start = 0, int len = 0);

        /*
         * Bind the uniform buffer to the OpenGL shader
         */
        virtual bool bindBuffer(Shader *, Renderer *, int locationOffset = 0);
        virtual void createBuffer(Renderer* renderer);
        virtual std::string makeShaderLayout();

        /**
         * Dump the contents of the shader uniforms to the log.
         * @param programID OpenGL program ID for shader
         * @param blockIndex OpenGL binding point
         */
        static void dump(GLuint programID, int blockIndex);

    protected:
        GLuint GLBuffer = 0;
    };

}
#endif
