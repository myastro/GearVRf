/*
---------------------------------------------------------------------------
Open Asset Import Library - Java Binding (jassimp)
---------------------------------------------------------------------------

Copyright (c) 2006-2012, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms, 
with or without modification, are permitted provided that the following 
conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------------------
*/
package org.gearvrf.jassimp;


import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;


public class AiMeshAnim {


    private final int SIZEOF_DOUBLE = Jassimp.NATIVE_DOUBLE_SIZE;


    AiMeshAnim(String nodeName, int numMeshMorphKeys, int numMorphTargets)
    {
        m_nodeName = nodeName;
        m_numMeshMorphKeys = numMeshMorphKeys;
        m_numMorphTargets = numMorphTargets;

        m_morphTargetWeights = ByteBuffer.allocateDirect(numMeshMorphKeys * (numMorphTargets + 1) * SIZEOF_DOUBLE);
        m_morphTargetWeights.order(ByteOrder.nativeOrder());
    }

    /**
     * @return  the morph animation keyframe data in the following timestamp blend weight format
     *
     *          t1, w, w, w, w, .....w, w, w
                t2, w, w, w, w, .....w, w, w
                t3, w, w, w, w, .....w, w, w
                t4, w, w, w, w, .....w, w, w
                .
                .
     *
     */
    public float[] getMorphAnimationKeys()
    {
        DoubleBuffer weights = m_morphTargetWeights.asDoubleBuffer();
        int temp = weights.remaining();
        double[] arr = new double[weights.remaining()];
        weights.get(arr);


        float[] farr = new float[arr.length];
        for (int i = 0 ; i < arr.length; i++)
        {
            farr[i] = (float) arr[i];
        }
        return farr;
    }


    /**
     * @return the number of morph targets in this mesh morph animation
     */
    public int getNumMorphTargets()
    {
        return m_numMorphTargets;
    }


    /**
     *
     * @return the name of the mesh of this animation
     */
    public String getNodeName()
    {
        return m_nodeName;
    }

    /**
     * Node name.
     */
    private final String m_nodeName;


    /**
     * Number of position keys.
     */
    private final int m_numMeshMorphKeys;

    /**
     * Number of morph targets.
     */
    private final int m_numMorphTargets;

    /**
     * Buffer with position keys.
     */
    private ByteBuffer m_morphTargetWeights;

}
