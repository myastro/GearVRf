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

/***************************************************************************
 * Containing data about how to render an object.
 ***************************************************************************/

#ifndef VULKAN_RENDER_DATA_H_
#define VULKAN_RENDER_DATA_H_

#include "engine/renderer/vulkan_renderer.h"
#include "objects/components/render_data.h"
#include "vulkan_headers.h"
#include "vulkan_shader.h"
#include "vulkan_vertex_buffer.h"
#include "vulkan_index_buffer.h"

typedef unsigned long Long;
namespace gvr
{
struct VulkanRenderPass : public RenderPass
{
    virtual ~VulkanRenderPass() {}
    VkPipeline m_pipeline = 0;
    std::vector<VkDescriptorSet> mDescriptorSets;
};


    /**
     * Vulkan implementation of RenderData.
     * Specializes handling of transform matrices.
     */
    class VulkanRenderData : public RenderData
    {
    public:
        VulkanRenderData() : RenderData()
        {
        }

        explicit VulkanRenderData(const RenderData &rdata) : RenderData(rdata)
        {
        }

        virtual ~VulkanRenderData() {}

        void createPipeline(VulkanRenderer* renderer, RenderSorter::Renderable& r, RenderState& rstate, VkRenderPass);

        VkPipeline getVKPipeline(int pass)
        {
            VulkanRenderPass* renderPass = static_cast<VulkanRenderPass*>(render_pass_list_[pass]);
            return renderPass->m_pipeline;

        }

        std::vector<VkDescriptorSet> getDescriptorSet(int pass)
        {
            VulkanRenderPass* renderPass = static_cast<VulkanRenderPass*>(render_pass_list_[pass]);
            return renderPass->mDescriptorSets;
        }
        void setPipeline(VkPipeline pipeline, int pass){
            VulkanRenderPass* renderPass = static_cast<VulkanRenderPass*>(render_pass_list_[pass]);
            renderPass->m_pipeline = pipeline;

        }
        void generateVbos(const std::string& descriptor, VulkanRenderer* renderer, Shader* shader){
            VulkanVertexBuffer* vbuf = static_cast<VulkanVertexBuffer*>(mesh_->getVertexBuffer());
            vbuf->generateVKBuffers(renderer->getCore(),shader);
            VulkanIndexBuffer* ibuf = static_cast< VulkanIndexBuffer*>(mesh_->getIndexBuffer());
            ibuf->generateVKBuffers(renderer->getCore());
        }

        VulkanRenderPass* getRenderPass(int pass){
            return static_cast<VulkanRenderPass*>(render_pass_list_[pass]);
        }

        VulkanRenderPass* getShadowRenderPass(){
            return shadowPass;
        }

        void setShadowRenderPass(VulkanRenderPass* sp){
            shadowPass = sp;
        }

        void bindToShader(Shader* shader, Renderer* renderer);
        bool isDirty(int pass){
            return isHashCodeDirty() || RenderData::isDirty();
        }
        void render(Shader* shader, VkCommandBuffer cmdBuffer, int curr_pass);
    private:
        //  VulkanRenderData(const VulkanRenderData& render_data);
        VulkanRenderData(VulkanRenderData&&);

        VulkanRenderData &operator=(const VulkanRenderData&);

        VulkanRenderData &operator=(VulkanRenderData&&);

    private:
//<<<<<<< HEAD
        //VulkanUniformBlock ubo  = nullptr;
        VulkanRenderPass * shadowPass = nullptr;
//=======
//        VulkanUniformBlock* ubo = nullptr;
//>>>>>>> bba42476... validate function for vulkan

    };

}


#endif
