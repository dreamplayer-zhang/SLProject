#include <SLECS.h>

#include <SLMaterial.h>
#include <SLGLGenericProgram.h>

#include <Instrumentor.h>

ECS::entity_id ECS::addEntity(ECS::World& world)
{
    ECS::entity_id result = world.entityCount;

    world.entityCount++;

    return result;
}

void ECS::addTreeNodeComponent(ECS::World& world,
                               entity_id   entityId,
                               entity_id   parentNodeId,
                               SLMat4f     om)
{
    world.entities[entityId].componentFlags |= ECS::ComponentType_TreeNode;
    world.entities[entityId].componentFlags |= ECS::ComponentType_Transform;
    world.treeNodeComponents[entityId].parentNodeId = parentNodeId;
    world.transformComponents[entityId].om          = om;
}

void ECS::addLightComponent(ECS::World&    world,
                            ECS::entity_id entityId,
                            SLVec4f        ambient,
                            SLVec4f        diffuse,
                            SLVec4f        specular,
                            float          spotCutoff,
                            float          spotCosCut,
                            float          spotExp,
                            float          kc,
                            float          kl,
                            float          kq,
                            bool32         isAttenuated)
{
    world.entities[entityId].componentFlags |= ECS::ComponentType_Light;

    world.lightComponents[entityId].ambient      = ambient;
    world.lightComponents[entityId].diffuse      = diffuse;
    world.lightComponents[entityId].specular     = specular;
    world.lightComponents[entityId].spotCutoff   = spotCutoff;
    world.lightComponents[entityId].spotCosCut   = spotCosCut;
    world.lightComponents[entityId].spotExp      = spotExp;
    world.lightComponents[entityId].kc           = kc;
    world.lightComponents[entityId].kl           = kl;
    world.lightComponents[entityId].kq           = kq;
    world.lightComponents[entityId].isAttenuated = isAttenuated;
}

void ECS::addRenderComponent(ECS::World&       world,
                             ECS::entity_id    entityId,
                             uint32_t          programId,
                             uint32_t          vaoId,
                             SLCol4f           ambient,
                             SLCol4f           diffuse,
                             SLCol4f           specular,
                             SLCol4f           emissive,
                             float             shininess,
                             uint32_t          texId,
                             SLMat4f           texMat,
                             SLGLPrimitiveType primitiveType,
                             uint32_t          numIndexes,
                             SLGLBufferType    indexDataType,
                             uint32_t          indexOffset)
{
    world.entities[entityId].componentFlags |= ECS::ComponentType_Render;

    world.renderComponents[entityId].programId     = programId;
    world.renderComponents[entityId].vaoId         = vaoId;
    world.renderComponents[entityId].ambient       = ambient;
    world.renderComponents[entityId].diffuse       = diffuse;
    world.renderComponents[entityId].specular      = specular;
    world.renderComponents[entityId].emissive      = emissive;
    world.renderComponents[entityId].shininess     = shininess;
    world.renderComponents[entityId].texId         = texId;
    world.renderComponents[entityId].texMat        = texMat;
    world.renderComponents[entityId].primitiveType = primitiveType;
    world.renderComponents[entityId].numIndexes    = numIndexes;
    world.renderComponents[entityId].indexDataType = indexDataType;
    world.renderComponents[entityId].indexOffset   = indexOffset;
}

void ECS::convertToComponents(SLNode*     root,
                              ECS::World& world,
                              SLVLight*   lights,
                              entity_id   parentNodeId)
{
    PROFILE_FUNCTION();

    ECS::entity_id nodeId = ECS::addEntity(world);
    ECS::addTreeNodeComponent(world, nodeId, parentNodeId, root->om());
    world.entities[nodeId].node = root;

    if (SLLight* slLightNode = dynamic_cast<SLLight*>(root))
    {
        addLightComponent(world,
                          nodeId,
                          slLightNode->ambient(),
                          slLightNode->diffuse(),
                          slLightNode->specular(),
                          slLightNode->spotCutOffDEG(),
                          slLightNode->spotCosCut(),
                          slLightNode->spotExponent(),
                          slLightNode->kc(),
                          slLightNode->kl(),
                          slLightNode->kq(),
                          slLightNode->isAttenuated());
    }

    if (root->mesh())
    {
        SLMesh* mesh = root->mesh();
        if (!mesh->vao().vaoID())
            mesh->generateVAO(mesh->vao());

        SLMaterial* mat = mesh->mat();
        if (!mat->program())
        {
            bool hasNrm = mat->textures().size() > 1 && mat->textures()[1]->texType() == TT_normal;
            bool hasAO  = mat->textures().size() > 2 && mat->textures()[2]->texType() == TT_ambientOcclusion;
            bool hasSM  = lights->size() > 0 && lights->at(0)->createsShadows();

            if (!mat->textures().empty())
            {
                if (hasNrm && hasAO && hasSM)
                    mat->program(SLGLDefaultProgPerPixBlinnTexNrmAOSM::instance());
                else if (hasNrm && hasAO)
                    mat->program(SLGLDefaultProgPerPixBlinnTexNrmAO::instance());
                else if (hasNrm && hasSM)
                    mat->program(SLGLDefaultProgPerPixBlinnTexNrmSM::instance());
                else if (hasNrm)
                    mat->program(SLGLDefaultProgPerPixBlinnTexNrm::instance());
                else if (hasSM)
                    mat->program(SLGLDefaultProgPerPixBlinnTexSM::instance());
                else
                    mat->program(SLGLDefaultProgPerVrtBlinnTex::instance());
            }
            else
            {
                if (hasSM)
                    mat->program(SLGLDefaultProgPerPixBlinnSM::instance());
                else
                    mat->program(SLGLDefaultProgPerVrtBlinn::instance());
            }
        }

        SLGLProgram* prog = mat->program();
        if (!prog->progID())
            prog->init(lights);

        for (SLuint texUnit = 0; texUnit < mat->textures().size(); ++texUnit)
        {
            SLchar name[100];
            mat->textures()[texUnit]->bindActive(texUnit);
        }

        SLMat4f  textureMat;
        uint32_t textureId = 0;
        if (mat->textures().size() > 0)
        {
            textureMat = mat->textures()[0]->tm();
            textureId  = mat->textures()[0]->texID();
        }

        addRenderComponent(world,
                           nodeId,
                           prog->progID(),
                           mesh->vao().vaoID(),
                           mat->ambient(),
                           mat->diffuse(),
                           mat->specular(),
                           mat->emissive(),
                           mat->shininess(),
                           textureId,
                           textureMat,
                           mesh->primitive(),
                           mesh->vao().numIndicesElements(),
                           BT_ushort,
                           0);
    }

    std::vector<SLNode*> children = root->findChildren<SLNode>("", false);

    for (SLNode* child : children)
    {
        ECS::convertToComponents(child, world, lights, nodeId);
    }
}

void ECS::convertToNodes(ECS::World& world)
{
    PROFILE_FUNCTION();

    for (int i = 0; i < world.entityCount; i++)
    {
        ECS::Entity& e = world.entities[i];
        if ((e.componentFlags & ComponentType_TreeNode) &&
            (e.componentFlags & ComponentType_Transform) &&
            e.node)
        {
            ECS::TransformComponent& tC = world.transformComponents[i];
            e.node->wm(tC.wm);
        }
    }
}

void ECS::transformUpdateSystem(ECS::World& world)
{
    int nodeIndices[MAX_ENTITY_COUNT];
    int nodeIndexCount   = 1;
    int currentNodeIndex = 0;
    nodeIndices[0]       = -1;

    while (currentNodeIndex < nodeIndexCount)
    {
        int currentParentIndex = nodeIndices[currentNodeIndex];
        currentNodeIndex++;

        for (int i = 0; i < world.entityCount; i++)
        {
            ECS::Entity& e = world.entities[i];
            if ((e.componentFlags & ComponentType_TreeNode) &&
                (e.componentFlags & ComponentType_Transform))
            {
                ECS::TreeNodeComponent& c = world.treeNodeComponents[i];
                if (c.parentNodeId == currentParentIndex)
                {
                    //printf("Tree node found with entity index %i for parent entity index %i\n", i, currentParentIndex);

                    ECS::TransformComponent& tC = world.transformComponents[i];
                    if (currentParentIndex < 0)
                        tC.wm = tC.om;
                    else
                        tC.wm = world.transformComponents[currentParentIndex].wm * tC.om;

                    bool nodeIndexInList = false;
                    for (int j = 0; j < nodeIndexCount; j++)
                    {
                        if (nodeIndices[j] == i)
                        {
                            nodeIndexInList = true;
                            break;
                        }
                    }

                    if (!nodeIndexInList)
                    {
                        nodeIndices[nodeIndexCount] = i;
                        nodeIndexCount++;
                    }
                }
            }
        }
    }

    //printf("End of system\n");
}

void ECS::renderSystem(ECS::World& world,
                       SLCol4f     globalAmbient,
                       SLfloat     gamma,
                       SLMat4f     viewMat,
                       SLMat4f     projectionMat)
{
    std::vector<std::tuple<RenderComponent, TransformComponent>> renderAndTransformComponents;
    for (int i = 0; i < world.entityCount; i++)
    {
        ECS::Entity& e = world.entities[i];

        if ((e.componentFlags & ComponentType_Render) &&
            (e.componentFlags & ComponentType_Transform))
        {
            renderAndTransformComponents.push_back(std::tuple<RenderComponent, TransformComponent>(world.renderComponents[i], world.transformComponents[i]));
        }
    }

    std::vector<std::tuple<LightComponent, TransformComponent>> lightAndTransformComponents;
    for (int i = 0; i < world.entityCount; i++)
    {
        ECS::Entity& e = world.entities[i];

        if ((e.componentFlags & ComponentType_Light) &&
            (e.componentFlags & ComponentType_Transform))
        {
            lightAndTransformComponents.push_back(std::tuple<LightComponent, TransformComponent>(world.lightComponents[i], world.transformComponents[i]));
        }
    }

    // TODO(dgj1): assert that number of lights < max lights

    struct
    {
        bool operator()(std::tuple<RenderComponent, TransformComponent> a, std::tuple<RenderComponent, TransformComponent> b) const
        {
            return std::get<RenderComponent>(a).programId < std::get<RenderComponent>(b).programId;
        }
    } sortByProgramId;

    std::sort(renderAndTransformComponents.begin(),
              renderAndTransformComponents.end(),
              sortByProgramId);

    float   oneOverGamma = 1.0f / gamma;
    SLMat4f viewRotMat(viewMat);
    viewRotMat.translation(0, 0, 0); // delete translation part, only rotation needed

    for (int i = 0; i < renderAndTransformComponents.size();)
    {
        RenderComponent rc        = std::get<RenderComponent>(renderAndTransformComponents.at(i));
        uint32_t        programId = rc.programId;

        ////////////////////////
        // 1) Apply Drawing Bits
        ////////////////////////

        // TODO(dgj1): actually do this... is there a way to
        // do this without passing SLSceneView?

        ///////////////////////////////////////
        // 2) Generate Vertex Array Object once
        ///////////////////////////////////////

        // NOTE(dgj1): this should be done when the component
        // is initialized

        /////////////////////////////
        // 3) Apply Uniform Variables
        /////////////////////////////

        SLint lightIsOn[SL_MAX_LIGHTS]; //!< flag if light is on
        //SLVec4f          lightPosWS[SL_MAX_LIGHTS];             //!< position of light in world space
        SLVec4f lightPosVS[SL_MAX_LIGHTS];    //!< position of light in view space
        SLVec4f lightAmbient[SL_MAX_LIGHTS];  //!< ambient light intensity (Ia)
        SLVec4f lightDiffuse[SL_MAX_LIGHTS];  //!< diffuse light intensity (Id)
        SLVec4f lightSpecular[SL_MAX_LIGHTS]; //!< specular light intensity (Is)
        //SLVec3f          lightSpotDirWS[SL_MAX_LIGHTS];         //!< spot direction in world space
        SLVec3f lightSpotDirVS[SL_MAX_LIGHTS];  //!< spot direction in view space
        SLfloat lightSpotCutoff[SL_MAX_LIGHTS]; //!< spot cutoff angle 1-180 degrees
        SLfloat lightSpotCosCut[SL_MAX_LIGHTS]; //!< cosine of spot cutoff angle
        SLfloat lightSpotExp[SL_MAX_LIGHTS];    //!< spot exponent
        SLVec3f lightAtt[SL_MAX_LIGHTS];        //!< att. factor (const,linear,quadratic)
        SLint   lightDoAtt[SL_MAX_LIGHTS];      //!< flag if att. must be calculated
                                                //SLint            lightCreatesShadows[SL_MAX_LIGHTS];    //!< flag if light creates shadows
                                                //SLint            lightDoSmoothShadows[SL_MAX_LIGHTS];   //!< flag if percentage-closer filtering is enabled
                                                //SLuint           lightSmoothShadowLevel[SL_MAX_LIGHTS]; //!< radius of area to sample
                                                //SLfloat          lightShadowMinBias[SL_MAX_LIGHTS];     //!< shadow mapping min. bias at 0 deg.
                                                //SLfloat          lightShadowMaxBias[SL_MAX_LIGHTS];     //!< shadow mapping max. bias at 90 deg.
                                                //SLint            lightUsesCubemap[SL_MAX_LIGHTS];       //!< flag if light has a cube shadow map
                                                //SLMat4f          lightSpace[SL_MAX_LIGHTS * 6];         //!< projection matrix of the light
                                                //SLGLDepthBuffer* lightShadowMap[SL_MAX_LIGHTS];         //!< pointers to depth-buffers for shadow mapping

        // Init to defaults
        for (SLint i = 0; i < SL_MAX_LIGHTS; ++i)
        {
            lightIsOn[i] = 0;
            //lightPosWS[i]      = SLVec4f(0, 0, 1, 1);
            lightPosVS[i]    = SLVec4f(0, 0, 1, 1);
            lightAmbient[i]  = SLCol4f::BLACK;
            lightDiffuse[i]  = SLCol4f::BLACK;
            lightSpecular[i] = SLCol4f::BLACK;
            //lightSpotDirWS[i]  = SLVec3f(0, 0, -1);
            lightSpotDirVS[i]  = SLVec3f(0, 0, -1);
            lightSpotCutoff[i] = 180.0f;
            lightSpotCosCut[i] = cos(Utils::DEG2RAD * lightSpotCutoff[i]);
            lightSpotExp[i]    = 1.0f;
            lightAtt[i].set(1.0f, 0.0f, 0.0f);
            lightDoAtt[i] = 0;
            /*for (SLint ii = 0; ii < 6; ++ii)
                lightSpace[i * 6 + ii] = SLMat4f();
            lightCreatesShadows[i]    = 0;
            lightDoSmoothShadows[i]   = 0;
            lightSmoothShadowLevel[i] = 1;
            lightShadowMinBias[i]     = 0.001f;
            lightShadowMaxBias[i]     = 0.008f;
            lightUsesCubemap[i]       = 0;
            lightShadowMap[i]         = nullptr;*/
        }

        // Fill up light property vectors
        for (int j = 0; j < lightAndTransformComponents.size(); j++)
        {
            std::tuple<LightComponent, TransformComponent> lightAndTransform = lightAndTransformComponents.at(j);

            LightComponent&     lc = std::get<LightComponent>(lightAndTransform);
            TransformComponent& tc = std::get<TransformComponent>(lightAndTransform);
            //SLShadowMap*   shadowMap = light->shadowMap();

            lightIsOn[i]  = lc.isOn;
            SLVec4f posWS = tc.wm.translation();
            //lightPosWS[i].set(posWS);
            SLVec4f posVS = viewMat * posWS;
            lightPosVS[i].set(posVS);
            lightAmbient[i].set(lc.ambient);
            lightDiffuse[i].set(lc.diffuse);
            lightSpecular[i].set(lc.specular);
            SLVec3f dirWS = SLVec3f(-tc.om.m(8), -tc.om.m(9), -tc.om.m(10));
            //lightSpotDirWS[i].set(dirWS);
            SLVec3f dirVS = viewRotMat.multVec(dirWS);
            lightSpotDirVS[i].set(dirVS);
            lightSpotCutoff[i] = lc.spotCutoff;
            lightSpotCosCut[i] = lc.spotCosCut;
            lightSpotExp[i]    = lc.spotExp;
            lightAtt[i]        = SLVec3f(lc.kc, lc.kl, lc.kq);
            lightDoAtt[i]      = lc.isAttenuated;
            /*lightCreatesShadows[i]    = light->createsShadows();
            lightDoSmoothShadows[i]   = light->doSoftShadows();
            lightSmoothShadowLevel[i] = light->softShadowLevel();
            lightShadowMinBias[i]     = light->shadowMinBias();
            lightShadowMaxBias[i]     = light->shadowMaxBias();
            lightUsesCubemap[i]       = shadowMap && shadowMap->useCubemap() ? 1 : 0;
            lightShadowMap[i]         = shadowMap && shadowMap->depthBuffer() ? shadowMap->depthBuffer() : nullptr;
            if (lightShadowMap[i])
                for (SLint ls = 0; ls < 6; ++ls)
                    lightSpace[i * 6 + ls] = shadowMap->mvp()[ls];
                    */
        }

        glUseProgram(rc.programId);
        GET_GL_ERROR;

        glUniform1f(glGetUniformLocation(rc.programId, "u_oneOverGamma"), oneOverGamma);
        glUniform4fv(glGetUniformLocation(rc.programId, "u_globalAmbi"), 1, (float*)&globalAmbient);
        GET_GL_ERROR;

        // Pass vectors as uniform vectors
        int nL = lightAndTransformComponents.size();
        glUniform1iv(glGetUniformLocation(rc.programId, "u_lightIsOn"), nL, (SLint*)&lightIsOn);
        GET_GL_ERROR;
        //glUniform4fv(glGetUniformLocation(c.programId, "u_lightPosWS"), nL, (SLfloat*)&lightPosWS);
        glUniform4fv(glGetUniformLocation(rc.programId, "u_lightPosVS"), nL, (SLfloat*)&lightPosVS);
        glUniform4fv(glGetUniformLocation(rc.programId, "u_lightAmbi"), nL, (SLfloat*)&lightAmbient);
        glUniform4fv(glGetUniformLocation(rc.programId, "u_lightDiff"), nL, (SLfloat*)&lightDiffuse);
        glUniform4fv(glGetUniformLocation(rc.programId, "u_lightSpec"), nL, (SLfloat*)&lightSpecular);
        glUniform3fv(glGetUniformLocation(rc.programId, "u_lightSpotDir"), nL, (SLfloat*)&lightSpotDirVS);
        glUniform1fv(glGetUniformLocation(rc.programId, "u_lightSpotDeg"), nL, (SLfloat*)&lightSpotCutoff);
        glUniform1fv(glGetUniformLocation(rc.programId, "u_lightSpotCos"), nL, (SLfloat*)&lightSpotCosCut);
        glUniform1fv(glGetUniformLocation(rc.programId, "u_lightSpotExp"), nL, (SLfloat*)&lightSpotExp);
        glUniform3fv(glGetUniformLocation(rc.programId, "u_lightAtt"), nL, (SLfloat*)&lightAtt);
        glUniform1iv(glGetUniformLocation(rc.programId, "u_lightDoAtt"), nL, (SLint*)&lightDoAtt);
        GET_GL_ERROR;
        /*glUniform1iv(glGetUniformLocation(c.programId, "u_lightDoSmoothShadows"), nL, (SLint*)&lightDoSmoothShadows);
        glUniform1iv(glGetUniformLocation(c.programId, "u_lightSmoothShadowLevel"), nL, (SLint*)&lightSmoothShadowLevel);
        glUniform1iv(glGetUniformLocation(c.programId, "u_lightUsesCubemap"), nL, (SLint*)&lightUsesCubemap);
        glUniformMatrix4fv(glGetUniformLocation(c.programId, "u_lightSpace"), nL * 6, (SLfloat*)&lightSpace);
        glUniform1iv(glGetUniformLocation(c.programId, "u_lightCreatesShadows"), nL, (SLint*)&lightCreatesShadows);
        glUniform1fv(glGetUniformLocation(c.programId, "u_lightShadowMinBias"), nL, (SLfloat*)&lightShadowMinBias);
        glUniform1fv(glGetUniformLocation(c.programId, "u_lightShadowMaxBias"), nL, (SLfloat*)&lightShadowMaxBias);*/

        glUniform4fv(glGetUniformLocation(rc.programId, "u_matAmbi"), 1, (float*)&rc.ambient);
        glUniform4fv(glGetUniformLocation(rc.programId, "u_matDiff"), 1, (float*)&rc.diffuse);
        glUniform4fv(glGetUniformLocation(rc.programId, "u_matSpec"), 1, (float*)&rc.specular);
        glUniform4fv(glGetUniformLocation(rc.programId, "u_matEmis"), 1, (float*)&rc.emissive);
        glUniform1f(glGetUniformLocation(rc.programId, "u_matShin"), rc.shininess);
        GET_GL_ERROR;
        /*program->uniform1f("u_matRough", _roughness);
        program->uniform1f("u_matMetal", _metalness);
        program->uniform1f("u_matKr", _kr);
        program->uniform1f("u_matKt", _kt);
        program->uniform1f("u_matKn", _kn);
        program->uniform1i("u_matGetsShadows", _getsShadows);*/
        glUniform1i(glGetUniformLocation(rc.programId, "u_matHasTexture"), rc.texId ? 1 : 0);

        // TODO(dgj1): this assumes the texture was already processed and loaded to the GPU
        glUniform1i(glGetUniformLocation(rc.programId, "u_matTexture0"), 0); // pass texture unit, not texId
        GET_GL_ERROR;

        glUniform1i(glGetUniformLocation(rc.programId, "u_camProjection"), 0);
        GET_GL_ERROR;
        /*glUniform1i(getGetUniformLocation(c.programId, "u_camStereoEye"), 0);
        glUniformMatrix3fv(getGetUniformLocation(c.programId, "u_camStereoColors"), 1, (SLfloat*)&_stereoColorFilter);
        // Pass fog parameters
        if (_fogColorIsBack)
            _fogColor = _background.avgColor();
        _fogStart = _clipNear;
        _fogEnd   = _clipFar;*/

        glUniform1i(glGetUniformLocation(rc.programId, "u_camFogIsOn"), 0);
        GET_GL_ERROR;
        /*glUniform1i("u_camFogMode", _fogMode);
        glUniform1f("u_camFogDensity", _fogDensity);
        glUniform1f("u_camFogStart", _fogStart);
        glUniform1f("u_camFogEnd", _fogEnd);
        glUniform4fv("u_camFogColor", 1, (SLfloat*)&_fogColor);
        glUniform1i("u_camFbWidth", _fbRect.width);
        glUniform1i("u_camFbHeight", _fbRect.height);*/

        GET_GL_ERROR;

        while (i < renderAndTransformComponents.size())
        {
            rc                    = std::get<RenderComponent>(renderAndTransformComponents.at(i));
            TransformComponent tc = std::get<TransformComponent>(renderAndTransformComponents.at(i));

            if (rc.programId != programId) break;

            i++;

            SLMat4f modelViewMat = viewMat;
            modelViewMat.multiply(tc.wm);

            SLMat4f modelViewProjectionMat = projectionMat;
            modelViewProjectionMat.multiply(modelViewMat);

            // 3.b) Pass the matrices to the shader program
            int mMatLoc = glGetUniformLocation(rc.programId, "u_mMatrix");
            glUniformMatrix4fv(mMatLoc, 1, false, (float*)&tc.wm);
            GET_GL_ERROR;
            glUniformMatrix4fv(glGetUniformLocation(rc.programId, "u_mvMatrix"), 1, false, (float*)&modelViewMat);
            GET_GL_ERROR;
            int mvpMatLoc = glGetUniformLocation(rc.programId, "u_mvpMatrix");
            glUniformMatrix4fv(mvpMatLoc, 1, false, (float*)&modelViewProjectionMat);
            GET_GL_ERROR;

            // 3.c) Build & pass inverse, normal & texture matrix only if needed
            //SLint locIM = sp->getUniformLocation("u_invMvMatrix");
            SLint locNM = glGetUniformLocation(rc.programId, "u_nMatrix");
            SLint locTM = glGetUniformLocation(rc.programId, "u_tMatrix");
            GET_GL_ERROR;

            /*if (locIM >= 0 && locNM >= 0)
            {
                stateGL->buildInverseAndNormalMatrix();
                sp->uniformMatrix4fv(locIM, 1, (const SLfloat*)stateGL->invModelViewMatrix());
                sp->uniformMatrix3fv(locNM, 1, (const SLfloat*)stateGL->normalMatrix());
            }
            else if (locIM >= 0)
            {
                stateGL->buildInverseMatrix();
                sp->uniformMatrix4fv(locIM, 1, (const SLfloat*)stateGL->invModelViewMatrix());
            }
            else */
            if (locNM >= 0)
            {
                SLMat3f normalMat = modelViewMat.mat3();
                normalMat.invert();
                normalMat.transpose();
                glUniformMatrix3fv(locNM, 1, false, (float*)&normalMat);
                GET_GL_ERROR;
            }
            if (locTM >= 0)
            {
                /*if (_mat->has3DTexture() && _mat->textures()[0]->autoCalcTM3D())
                    calcTex3DMatrix(node);
                else*/
                glUniformMatrix4fv(locTM, 1, false, (SLfloat*)&rc.texMat);
                GET_GL_ERROR;
            }

            ///////////////////////////////
            // 4): Finally do the draw call
            ///////////////////////////////

            glBindVertexArray(rc.vaoId);
            GET_GL_ERROR;

            // Do the draw call with indices
            /*if (rc.numIndexes == 0)
                numIndexes = _numIndicesElements;*/

            SLuint indexTypeSize = SLGLVertexBuffer::sizeOfType(rc.indexDataType);

            /////////////////////////////////////////////////////////////////////
            glDrawElements(rc.primitiveType,
                           (SLsizei)rc.numIndexes,
                           rc.indexDataType,
                           (void*)(size_t)(rc.indexOffset * (SLuint)indexTypeSize));
            /////////////////////////////////////////////////////////////////////

            GET_GL_ERROR;

            glBindVertexArray(0);
            GET_GL_ERROR;
        }
    }
}
