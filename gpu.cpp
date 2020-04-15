/*!
 * @file
 * @brief This file contains implementation of gpu
 *
 * @author Tomáš Milet, imilet@fit.vutbr.cz
 */

#include <student/gpu.hpp>
#include<algorithm>


 /// \addtogroup gpu_init
 /// @{

 /**
  * @brief Constructor of GPU
  */
GPU::GPU() {


}

/**
 * @brief Destructor of GPU
 */
GPU::~GPU() {



}

/// @}

/** \addtogroup buffer_tasks 01. Implementace obslužných funkcí pro buffery
 * @{
 */

 /**
  * @brief This function allocates buffer on GPU.
  *
  * @param size size in bytes of new buffer on GPU.
  *
  * @return unique identificator of the buffer
  */


BufferID GPU::createBuffer(uint64_t size) {
    Buffer* tmp = new Buffer();
    if (tmp == nullptr)
    {
        return emptyID;
    }
    //set address as ID - easier access to buffer in other methods
  //  tmp->ID = reinterpret_cast<BufferID>(tmp);
    tmp->data = new uint8_t[size];
    if (tmp->data == nullptr)
    {
        delete tmp;
        return emptyID;
    }
    tmp->size = size;

    BufferID id = reinterpret_cast<BufferID>(tmp);
    buffers.insert({ id, tmp });
    return id;
}

/**
 * @brief This function frees allocated buffer on GPU.
 *
 * @param buffer buffer identificator
 */
void GPU::deleteBuffer(BufferID buffer) {
    auto todelete = buffers.find(buffer);
    if (todelete != buffers.end())
    {
        delete[] buffers.find(buffer)->second->data;
        delete buffers.find(buffer)->second;
        buffers.erase(buffer);

    }
}

/**
 * @brief This function uploads data to selected buffer on the GPU
 *
 * @param buffer buffer identificator
 * @param offset specifies the offset into the buffer's data
 * @param size specifies the size of buffer that will be uploaded
 * @param data specifies a pointer to new data
 */
void GPU::setBufferData(BufferID buffer, uint64_t offset, uint64_t size, void const* data) {
    if (!isBuffer(buffer))
        return;
    //ID = address
    Buffer* buff = buffers.find(buffer)->second;
    if (size + offset > buff->size)
        return;
    //uint8_t* - size is in byte, 1 step in std::copy = byte
    std::copy((uint8_t*)data, ((uint8_t*)(data)) + size, (uint8_t*)(buff->data + offset));

}

/**
 * @brief This function downloads data from GPU.
 *
 * @param buffer specfies buffer
 * @param offset specifies the offset into the buffer from which data will be returned, measured in bytes.
 * @param size specifies data size that will be copied
 * @param data specifies a pointer to the location where buffer data is returned.
 */
void GPU::getBufferData(BufferID buffer, uint64_t offset, uint64_t size, void* data)
{
    if (!isBuffer(buffer))
        return;
    //ID = address
    Buffer* buff = buffers.find(buffer)->second;
    if (size + offset > buff->size)
        return;
    uint8_t* buffer_start = (uint8_t*)(buff->data + offset);
    //uint8_t* - size is in byte, 1 step in std::copy = byte
    std::copy(buffer_start, buffer_start + size, (uint8_t*)data);
}

/**
 * @brief This function tests if buffer exists
 *
 * @param buffer selected buffer id
 *
 * @return true if buffer points to existing buffer on the GPU.
 */
bool GPU::isBuffer(BufferID buffer) {
    return buffers.find(buffer) != buffers.end();
}

/// @}

/**
 * \addtogroup vertexpuller_tasks 02. Implementace obslužných funkcí pro vertex puller
 * @{
 */

 /**
  * @brief This function creates new vertex puller settings on the GPU,
  *
  * @return unique vertex puller identificator
  */
ObjectID GPU::createVertexPuller() {

    VertexPuller* table = new VertexPuller();
    VertexPullerID tableID = reinterpret_cast<VertexPullerID>(table);
    vertexPullers.insert({ tableID, table });
    return tableID;
}

/**
 * @brief This function deletes vertex puller settings
 *
 * @param vao vertex puller identificator
 */
void     GPU::deleteVertexPuller(VertexPullerID vao) {
    auto todelete = reinterpret_cast<VertexPuller*>(vao);
    delete todelete;
    vertexPullers.erase(vao);
}

/**
 * @brief This function sets one vertex puller reading head.
 *
 * @param vao identificator of vertex puller
 * @param head id of vertex puller head
 * @param type type of attribute
 * @param stride stride in bytes
 * @param offset offset in bytes
 * @param buffer id of buffer
 */
void     GPU::setVertexPullerHead(VertexPullerID vao, uint32_t head, AttributeType type, uint64_t stride, uint64_t offset, BufferID buffer) {
    VertexPuller* vpTable = reinterpret_cast<VertexPuller*>(vao);
    vpTable->heads.at(head) = { buffer, offset, stride, type,false };
}

/**
 * @brief This function sets vertex puller indexing.
 *
 * @param vao vertex puller id
 * @param type type of index
 * @param buffer buffer with indices
 */
void     GPU::setVertexPullerIndexing(VertexPullerID vao, IndexType type, BufferID buffer) {
    VertexPuller* vertexPuller = reinterpret_cast<VertexPuller*>(vao);
    vertexPuller->indexing.indexType = type;
    vertexPuller->indexing.bufferID = buffer;
}

/**
 * @brief This function enables vertex puller's head.
 *
 * @param vao vertex puller
 * @param head head id
 */
void     GPU::enableVertexPullerHead(VertexPullerID vao, uint32_t head) {
    VertexPuller* vpTable = reinterpret_cast<VertexPuller*>(vao);
    vpTable->heads.at(head).enabled = true;
}

/**
 * @brief This function disables vertex puller's head
 *
 * @param vao vertex puller id
 * @param head head id
 */
void     GPU::disableVertexPullerHead(VertexPullerID vao, uint32_t head) {
    VertexPuller* vpTable = reinterpret_cast<VertexPuller*>(vao);
    vpTable->heads.at(head).enabled = false;
}

/**
 * @brief This function selects active vertex puller.
 *
 * @param vao id of vertex puller
 */
void     GPU::bindVertexPuller(VertexPullerID vao) {
    activeVertexPullerID = vao;
}

/**
 * @brief This function deactivates vertex puller.
 */
void     GPU::unbindVertexPuller() {

    activeVertexPullerID = emptyID;
}

/**
 * @brief This function tests if vertex puller exists.
 *
 * @param vao vertex puller
 *
 * @return true, if vertex puller "vao" exists
 */
bool     GPU::isVertexPuller(VertexPullerID vao) {
    return vertexPullers.find(vao) != vertexPullers.end();
}

/// @}

/** \addtogroup program_tasks 03. Implementace obslužných funkcí pro shader programy
 * @{
 */

 /**
  * @brief This function creates new shader program.
  *
  * @return shader program id
  */
ProgramID        GPU::createProgram() {
    ShaderProgram* program = new ShaderProgram();
    ProgramID id = reinterpret_cast<ProgramID>(program);
    programs.insert({ id, program });
    return id;
}

/**
 * @brief This function deletes shader program
 *
 * @param prg shader program id
 */
void             GPU::deleteProgram(ProgramID prg) {
    auto todelete = programs.find(prg);
    //  delete todelete->second;
    programs.erase(prg);
}

/**
 * @brief This function attaches vertex and frament shader to shader program.
 *
 * @param prg shader program
 * @param vs vertex shader
 * @param fs fragment shader
 */
void             GPU::attachShaders(ProgramID prg, VertexShader vs, FragmentShader fs) {
    ShaderProgram* program = reinterpret_cast<ShaderProgram*>(prg);
    program->fragmentShader = fs;
    program->vertexShader = vs;
}

/**
 * @brief This function selects which vertex attributes should be interpolated during rasterization into fragment attributes.
 *
 * @param prg shader program
 * @param attrib id of attribute
 * @param type type of attribute
 */
void             GPU::setVS2FSType(ProgramID prg, uint32_t attrib, AttributeType type) {
    ShaderProgram* program = reinterpret_cast<ShaderProgram*>(prg);
    program->attributes.at(attrib) = type;

}

/**
 * @brief This function actives selected shader program
 *
 * @param prg shader program id
 */
void             GPU::useProgram(ProgramID prg) {
    activeProgramID = prg;

}

/**
 * @brief This function tests if selected shader program exists.
 *
 * @param prg shader program
 *
 * @return true, if shader program "prg" exists.
 */
bool             GPU::isProgram(ProgramID prg) {


    return programs.find(prg) != programs.end();
}

/**
 * @brief This function sets uniform value (1 float).
 *
 * @param prg shader program
 * @param uniformId id of uniform value (number of uniform values is stored in maxUniforms variable)
 * @param d value of uniform variable
 */
void             GPU::programUniform1f(ProgramID prg, uint32_t uniformId, float     const& d) {

    ShaderProgram* program = reinterpret_cast<ShaderProgram*>(prg);
    if (uniformId < maxUniforms)
    {
        program->uniforms.uniform[uniformId].v1 = d;
    }
}

/**
 * @brief This function sets uniform value (2 float).
 *
 * @param prg shader program
 * @param uniformId id of uniform value (number of uniform values is stored in maxUniforms variable)
 * @param d value of uniform variable
 */
void             GPU::programUniform2f(ProgramID prg, uint32_t uniformId, glm::vec2 const& d) {
    ShaderProgram* program = reinterpret_cast<ShaderProgram*>(prg);
    if (uniformId < maxUniforms)
    {
        program->uniforms.uniform[uniformId].v2 = d;
    }
}

/**
 * @brief This function sets uniform value (3 float).
 *
 * @param prg shader program
 * @param uniformId id of uniform value (number of uniform values is stored in maxUniforms variable)
 * @param d value of uniform variable
 */
void             GPU::programUniform3f(ProgramID prg, uint32_t uniformId, glm::vec3 const& d) {
    ShaderProgram* program = reinterpret_cast<ShaderProgram*>(prg);
    if (uniformId < maxUniforms)
    {
        program->uniforms.uniform[uniformId].v3 = d;
    }
}

/**
 * @brief This function sets uniform value (4 float).
 *
 * @param prg shader program
 * @param uniformId id of uniform value (number of uniform values is stored in maxUniforms variable)
 * @param d value of uniform variable
 */
void             GPU::programUniform4f(ProgramID prg, uint32_t uniformId, glm::vec4 const& d) {
    ShaderProgram* program = reinterpret_cast<ShaderProgram*>(prg);
    if (uniformId < maxUniforms)
    {
        program->uniforms.uniform[uniformId].v4 = d;
    }
}

/**
 * @brief This function sets uniform value (4 float).
 *
 * @param prg shader program
 * @param uniformId id of uniform value (number of uniform values is stored in maxUniforms variable)
 * @param d value of uniform variable
 */
void             GPU::programUniformMatrix4f(ProgramID prg, uint32_t uniformId, glm::mat4 const& d) {
    /// Místo 1 floatu nahrává matici 4x4 (16 floatů).
    ShaderProgram* program = reinterpret_cast<ShaderProgram*>(prg);
    if (uniformId < maxUniforms)
    {
        program->uniforms.uniform[uniformId].m4 = d;
    }
}

/// @}





/** \addtogroup framebuffer_tasks 04. Implementace obslužných funkcí pro framebuffer
 * @{
 */

 /**
  * @brief This function creates framebuffer on GPU.
  *
  * @param width width of framebuffer
  * @param height height of framebuffer
  */
void GPU::createFramebuffer(uint32_t width, uint32_t height) {
    frameBuffer.colorBuffer.resize(((size_t)width * height) + size_t(width));
    frameBuffer.depthBuffer.resize(((size_t)width * height) + size_t(width));
    frameBuffer.width = width;
    frameBuffer.height = height;
}

/**
 * @brief This function deletes framebuffer.
 */
void GPU::deleteFramebuffer() {
    frameBuffer.colorBuffer.clear();
    frameBuffer.colorBuffer.shrink_to_fit();
    frameBuffer.depthBuffer.clear();
    frameBuffer.depthBuffer.shrink_to_fit();

}

/**
 * @brief This function resizes framebuffer.
 *
 * @param width new width of framebuffer
 * @param height new heght of framebuffer
 */
void     GPU::resizeFramebuffer(uint32_t width, uint32_t height) {
    frameBuffer.depthBuffer.resize(((size_t)width * height) + size_t(width));
    frameBuffer.colorBuffer.resize(((size_t)width * height) + size_t(width));

    frameBuffer.width = width;
    frameBuffer.height = height;


}

/**
 * @brief This function returns pointer to color buffer.
 *
 * @return pointer to color buffer
 */
uint8_t* GPU::getFramebufferColor() {
    return (uint8_t*)&frameBuffer.colorBuffer[0];
}

/**
 * @brief This function returns pointer to depth buffer.
 *
 * @return pointer to dept buffer.
 */
float* GPU::getFramebufferDepth() {
    float* value = &frameBuffer.depthBuffer[0];
    return value;
}

/**
 * @brief This function returns width of framebuffer
 *
 * @return width of framebuffer
 */
uint32_t GPU::getFramebufferWidth() {
    return frameBuffer.width;
}

/**
 * @brief This function returns height of framebuffer.
 *
 * @return height of framebuffer
 */
uint32_t GPU::getFramebufferHeight() {
    return frameBuffer.height;
}

/// @}

/** \addtogroup draw_tasks 05. Implementace vykreslovacích funkcí
 * Bližší informace jsou uvedeny na hlavní stránce dokumentace.
 * @{
 */

 /**
  * @brief This functino clears framebuffer.
  *
  * @param r red channel
  * @param g green channel
  * @param b blue channel
  * @param a alpha channel
  */
void            GPU::clear(float r, float g, float b, float a) {
    uint8_t red = r >= 1.f ? 255 : (uint8_t)(255.f * r);
    uint8_t green = g >= 1.f ? 255 : (uint8_t)(255.f * g);
    uint8_t blue = b >= 1.f ? 255 : (uint8_t)(255.f * b);
    uint8_t alpha = a >= 1.f ? 255 : (uint8_t)(255.f * a);
    for (Color& pixel : frameBuffer.colorBuffer)
    {
        pixel.r = red;
        pixel.g = green;
        pixel.b = blue;
        pixel.a = alpha;
    }
    for (float& depth : frameBuffer.depthBuffer)
    {
        depth = std::numeric_limits<float>::infinity();
    }

}


void            GPU::drawTriangles(uint32_t  nofVertices) {
    /// \todo Tato funkce vykreslí trojúhelníky podle daného nastavení.<br>
    /// Vrcholy se budou vybírat podle nastavení z aktivního vertex pulleru (pomocí bindVertexPuller).<br>
    /// Vertex shader a fragment shader se zvolí podle aktivního shader programu (pomocí useProgram).<br>
    /// Parametr "nofVertices" obsahuje počet vrcholů, který by se měl vykreslit (3 pro jeden trojúhelník).<br>
      //active vertex puller

   //assemble triangles from buffers data
    if (nofVertices <= 0)
    {
        return;
    }
    std::vector<Triangle>assembledTriangles = assembleTriangles(nofVertices / 3);
    //clip triangles
    if (assembledTriangles.size() <= 0)
        return;
    std::vector<Triangle>clippedTriangles = clipTriangles(assembledTriangles);
    //normalize coords and transform to viewport 
    if (clippedTriangles.size() <= 0)
        return;
    viewportTransAndNormalize(clippedTriangles);
    rasterize(clippedTriangles);
}






std::vector<GPU::Triangle> GPU::assembleTriangles(uint32_t trianglesCount)
{

    std::vector<Triangle> triangles;
    //used for vertexPuller heads offset / indexing - starts at 0
    Triangle sad;
    uint32_t vertexShaderInvocations = 0;
    for (unsigned i = 0; i < (trianglesCount); i++)
    {
        Triangle newTriangle;
        for (unsigned j = 0; j < 3; j++)
        {
            newTriangle.triangleVertexes[j] = vertexProcessor(vertexShaderInvocations++);
        }

        triangles.push_back(newTriangle);
    }
    return triangles;
}
OutVertex GPU::vertexProcessor(uint32_t vertexNumber)
{
    InVertex vertexToProcess = vertexPullerRead(vertexNumber);

    OutVertex processedVertex;
    for (uint8_t i = 0; i < 3; i++)
    {
        processedVertex.gl_Position[i] = std::numeric_limits<float>::infinity() * -1.f;
    }
    processedVertex.gl_Position.w = 1;


    //program with activeProgramID doesnt exist
    if (!isProgram(activeProgramID) || vertexToProcess.gl_VertexID == emptyID)
    {
        return processedVertex;
    }
    //get active progoram pointer from ID
    ShaderProgram* activeProgram = programs.find(activeProgramID)->second;
    if (activeProgram->vertexShader == NULL)
    {
        return processedVertex;
    }
    //call assigned vertexShader function
    activeProgram->vertexShader(processedVertex, vertexToProcess, activeProgram->uniforms);
    return processedVertex;
}
InVertex GPU::vertexPullerRead(uint32_t vertexNumber)
{
    InVertex vertex;
    vertex.gl_VertexID = emptyID;
    //no active vp
    if (!isVertexPuller(activeVertexPullerID))
        return vertex;
    VertexPuller* activeVP = vertexPullers.find(activeVertexPullerID)->second;
    if (activeVP->indexing.bufferID != 0)
    {
        uint32_t indexSize = (uint8_t)(activeVP->indexing.indexType);
        vertex.gl_VertexID = 0;
        getBufferData(activeVP->indexing.bufferID, (uint32_t)(vertexNumber * indexSize), indexSize, &vertex.gl_VertexID);
    }
    else
    {
        vertex.gl_VertexID = vertexNumber;
    }
    for (size_t i = 0; i < maxAttributes; i++)
    {
        if (activeVP->heads.at(i).enabled)
        {
            BufferID bufID = activeVP->heads.at(i).ID;
            uint64_t offset = activeVP->heads.at(i).offset + activeVP->heads.at(i).stride * vertex.gl_VertexID;
            uint64_t size = sizeof(float) * (uint64_t)activeVP->heads.at(i).attrType;
            void* destination = &vertex.attributes[i];
            getBufferData(bufID, offset, size, destination);
        }
    }
    return vertex;
}
std::vector<GPU::Triangle> GPU::clipTriangles(std::vector<GPU::Triangle> triangles)
{
    uint64_t trianglesCount = triangles.size();
    std::vector<GPU::Triangle> clippedTriangles;
    for (auto triangle : triangles)
    {
        uint8_t vertexesOutCount = 0;
        for (auto vertex : triangle.triangleVertexes)
        {
            if (!inViewSpace(vertex))
                vertexesOutCount++;
        }

        if (vertexesOutCount == 0)
        {
            clippedTriangles.push_back(triangle);
            continue;
        }
        else if (vertexesOutCount == 1)
        {
            //2 triangles from 1

            Triangle newTriangle1;
            Triangle newTriangle2;
            if (!inViewSpace(triangle.triangleVertexes[0]))
            {
                //index 0 out

                //newTriangle1.triangleVertexes[1] = newTriangle2.triangleVertexes[1] = triangle.triangleVertexes[1];
                newTriangle1.triangleVertexes[1] = newTriangle2.triangleVertexes[2] = triangle.triangleVertexes[2];
                newTriangle1.triangleVertexes[2] = newTriangle2.triangleVertexes[1] = cutEdge(triangle.triangleVertexes[0], triangle.triangleVertexes[1]);

                newTriangle2.triangleVertexes[0] = triangle.triangleVertexes[1];
                newTriangle1.triangleVertexes[0] = cutEdge(triangle.triangleVertexes[0], triangle.triangleVertexes[2]);
            }
            else if (!inViewSpace(triangle.triangleVertexes[1]))
            {
                //out index 1
                newTriangle1.triangleVertexes[1] = newTriangle2.triangleVertexes[2] = triangle.triangleVertexes[2];
                newTriangle1.triangleVertexes[2] = newTriangle2.triangleVertexes[1] = cutEdge(triangle.triangleVertexes[0], triangle.triangleVertexes[1]);


                newTriangle2.triangleVertexes[0] = triangle.triangleVertexes[0];
                newTriangle1.triangleVertexes[0] = cutEdge(triangle.triangleVertexes[1], triangle.triangleVertexes[2]);
            }
            else if (!inViewSpace(triangle.triangleVertexes[2]))
            {
                //out index 2
                newTriangle1.triangleVertexes[1] = newTriangle2.triangleVertexes[2] = triangle.triangleVertexes[1];
                newTriangle1.triangleVertexes[2] = newTriangle2.triangleVertexes[1] = cutEdge(triangle.triangleVertexes[0], triangle.triangleVertexes[2]);


                newTriangle1.triangleVertexes[0] = triangle.triangleVertexes[0];
                newTriangle2.triangleVertexes[0] = cutEdge(triangle.triangleVertexes[1], triangle.triangleVertexes[2]);
            }


            clippedTriangles.push_back(newTriangle1);
            clippedTriangles.push_back(newTriangle2);
        }
        else if (vertexesOutCount == 2)
        {
            Triangle newTriangle;
            //1 triangle with different vertex coords
            if (inViewSpace(triangle.triangleVertexes[0]))
            {
                //index 1 and 2 out 
                newTriangle.triangleVertexes[0] = triangle.triangleVertexes[0];
                newTriangle.triangleVertexes[1] = cutEdge(triangle.triangleVertexes[0], triangle.triangleVertexes[1]);
                newTriangle.triangleVertexes[2] = cutEdge(triangle.triangleVertexes[0], triangle.triangleVertexes[2]);

            }
            else if (inViewSpace(triangle.triangleVertexes[1]))
            {
                //index 0 and 2 out
                newTriangle.triangleVertexes[1] = triangle.triangleVertexes[1];


                newTriangle.triangleVertexes[0] = cutEdge(triangle.triangleVertexes[1], triangle.triangleVertexes[0]);
                newTriangle.triangleVertexes[2] = cutEdge(triangle.triangleVertexes[1], triangle.triangleVertexes[2]);
            }
            else
            {
                //index 0 and 1 out
                newTriangle.triangleVertexes[2] = triangle.triangleVertexes[2];


                newTriangle.triangleVertexes[0] = cutEdge(triangle.triangleVertexes[2], triangle.triangleVertexes[0]);
                newTriangle.triangleVertexes[1] = cutEdge(triangle.triangleVertexes[2], triangle.triangleVertexes[1]);

            }
            clippedTriangles.push_back(newTriangle);

        }
        //if 3 vertexes out - ignore triangle (dont add to clipped triangles vector)
        //next triangle counter reset
        vertexesOutCount = 0;
    }
    return clippedTriangles;
}
bool GPU::inViewSpace(OutVertex vertex)
{
    return ((vertex.gl_Position.w * -1) <= vertex.gl_Position.z);
}
OutVertex GPU::cutEdge(OutVertex vertexInside, OutVertex vertexOutside)
{
    OutVertex newVertex;
    //parameter t for edge parametric equation
    float t = ((float)(-1 * vertexInside.gl_Position.w - vertexInside.gl_Position.z)) / (vertexOutside.gl_Position.w - vertexInside.gl_Position.w + vertexOutside.gl_Position.z - vertexInside.gl_Position.z);
    //linear interpolation of x,y,z,w coords
    for (uint8_t i = 0; i < 4; i++)
    {
        newVertex.gl_Position[i] = vertexInside.gl_Position[i] + t * (vertexOutside.gl_Position[i] - vertexInside.gl_Position[i]);
    }

    //attributes interpolation
    for (Attribute attribute : newVertex.attributes)
    {
        attribute.v4 = (t * vertexOutside.attributes->v4) + ((1 - t) * vertexInside.attributes->v4);
    }
    return newVertex;
}
void GPU::viewportTransAndNormalize(std::vector<GPU::Triangle>& triangles)
{
    for (auto& triangle : triangles)
    {
        for (auto& vertex : triangle.triangleVertexes)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                vertex.gl_Position[i] = vertex.gl_Position[i] / vertex.gl_Position[3];

            }
            vertex.gl_Position.x += 1;
            vertex.gl_Position.y += 1;
            vertex.gl_Position.x *= (float)getFramebufferWidth() / 2.0f;
            vertex.gl_Position.y *= (float)getFramebufferHeight() / 2.0f;
        }
    }
}

OutFragment GPU::fragmentProcessor(InFragment fragment)
{
    ShaderProgram* program = programs.find(activeProgramID)->second;
    OutFragment output;
    program->fragmentShader(output, fragment, program->uniforms);
    return output;
}

bool GPU::depthTest(uint32_t x, uint32_t y, float depth)
{
    float* p_depthBuffer = getFramebufferDepth();
    return depth < p_depthBuffer[getFramebufferWidth() * y + x];
}
void GPU::perFragment(uint32_t x, uint32_t y, float depth, OutFragment color)
{
    if (depthTest(x, y, depth))
    {
        float* p_depthBuffer = getFramebufferDepth();
        uint8_t* p_colorBuffer = getFramebufferColor();
        //overwrite depth
        p_depthBuffer[y * getFramebufferWidth() + x] = depth;
        //overwrite each color part (RGBA) and normalize to <0,1> range
        for (uint32_t i = 0; i < 4; i++)
        {

            p_colorBuffer[(y * getFramebufferWidth() + x) * 4 + i] = (color.gl_FragColor[i] >= 1.f) ? 255 : (uint8_t)((color.gl_FragColor[i]) * 255.f);
        }
    }
}

InFragment GPU::assembleInFragment(uint64_t x, uint64_t y, GPU::Triangle triangle)
{
    std::array<std::array<float, 2>, 3> triangleVertexesCoords =
    {
        triangle.triangleVertexes[0].gl_Position.x, triangle.triangleVertexes[0].gl_Position.y,
        triangle.triangleVertexes[1].gl_Position.x, triangle.triangleVertexes[1].gl_Position.y,
        triangle.triangleVertexes[2].gl_Position.x, triangle.triangleVertexes[2].gl_Position.y
    };
    float mainTriangleArea = triangleArea(triangleVertexesCoords);
    //lambda 0
    float midPixelX = (float)x + 0.5f;
    float midPixelY = (float)y + 0.5f;
    triangleVertexesCoords[0] = { midPixelX, midPixelY };
    float lambda0 = triangleArea(triangleVertexesCoords) / mainTriangleArea;
    //lambda 1
    triangleVertexesCoords[1] = { triangle.triangleVertexes[0].gl_Position.x, triangle.triangleVertexes[0].gl_Position.y };
    float lambda1 = triangleArea(triangleVertexesCoords) / mainTriangleArea;
    //lambda 2
    triangleVertexesCoords[2] = { triangle.triangleVertexes[1].gl_Position.x, triangle.triangleVertexes[1].gl_Position.y };
    float lambda2 = triangleArea(triangleVertexesCoords) / mainTriangleArea;





    std::array<float, 3> lambdas = { lambda0, lambda1, lambda2 };
    std::array<glm::vec4, 3> triangleCoords = { triangle.triangleVertexes[0].gl_Position,triangle.triangleVertexes[1].gl_Position, triangle.triangleVertexes[2].gl_Position };

    float depth = (((triangleCoords[0].z * lambdas[0]) / triangleCoords[0].w) + ((triangleCoords[1].z * lambdas[1]) / triangleCoords[1].w) + ((triangleCoords[2].z * lambdas[2]) / triangleCoords[2].w))
        / ((lambdas[0] / triangleCoords[0].w) + (lambdas[1] / triangleCoords[1].w) + (lambdas[2] / triangleCoords[2].w));

    InFragment fragment;
    fragment.gl_FragCoord.x = midPixelX;
    fragment.gl_FragCoord.y = midPixelY;

    fragment.gl_FragCoord.z = depth;
    ShaderProgram* program = programs.find(activeProgramID)->second;

    for (uint8_t i = 0; i < maxAttributes; i++)
    {
        std::array<glm::vec4, 3> attributes = { triangle.triangleVertexes[0].attributes[i].v4, triangle.triangleVertexes[1].attributes[i].v4, triangle.triangleVertexes[2].attributes[i].v4 };
        if (program->attributes[i] != AttributeType::EMPTY)
        {
            for (uint8_t j = 0; j < (uint8_t)program->attributes[i]; j++)
            {
                fragment.attributes[i].v4[j] = (((attributes[0][j] * lambdas[0]) / triangleCoords[0].w) + ((attributes[1][j] * lambdas[1]) / triangleCoords[1].w) + ((attributes[2][j] * lambdas[2]) / triangleCoords[2].w))
                    / ((lambdas[0] / triangleCoords[0].w) + (lambdas[1] / triangleCoords[1].w) + (lambdas[2] / triangleCoords[2].w));
            }
        }
    }
    fragment.gl_FragCoord.w = 1;
    return fragment;
}
float GPU::triangleArea(std::array<std::array<float, 2>, 3> edgeVectors)
{
    std::array<float, 3> edgeLengths;
    for (size_t i = 0; i < edgeVectors.size(); i++)
    {
        float xDifference = edgeVectors[i][0] - edgeVectors[(i + 1) % 3][0];
        float yDifference = edgeVectors[i][1] - edgeVectors[(i + 1) % 3][1];
        edgeLengths[i] = glm::length(glm::vec2(xDifference, yDifference));
    }
    float s = (edgeLengths[0] + edgeLengths[1] + edgeLengths[2]) / 2.f;
    return std::sqrt(s * (s - edgeLengths[0]) * (s - edgeLengths[1]) * (s - edgeLengths[2]));

}

void GPU::rasterize(std::vector<GPU::Triangle> triangles)
{
    for (auto triangle : triangles)
    {
        //area for pixel iteration
        float maxX_float = (std::max({ triangle.triangleVertexes[0].gl_Position.x, triangle.triangleVertexes[1].gl_Position.x , triangle.triangleVertexes[2].gl_Position.x }));
        float minX_float = (std::min({ triangle.triangleVertexes[0].gl_Position.x, triangle.triangleVertexes[1].gl_Position.x , triangle.triangleVertexes[2].gl_Position.x }));
        float maxY_float = (std::max({ triangle.triangleVertexes[0].gl_Position.y, triangle.triangleVertexes[1].gl_Position.y , triangle.triangleVertexes[2].gl_Position.y }));
        float minY_float = (std::min({ triangle.triangleVertexes[0].gl_Position.y, triangle.triangleVertexes[1].gl_Position.y , triangle.triangleVertexes[2].gl_Position.y }));


        //clip by framBuffer resolution
        int minX = (int)std::max(0.f, minX_float);
        int maxX = (int)std::min((float)getFramebufferWidth(), maxX_float);
        int minY = (int)std::max(0.f, minY_float);
        int maxY = (int)std::min((float)getFramebufferHeight(), maxY_float);


      
        //a and b parametes of normal vector for general line equation
        float a1 = triangle.triangleVertexes[0].gl_Position.y - triangle.triangleVertexes[1].gl_Position.y;
        float a2 = triangle.triangleVertexes[1].gl_Position.y - triangle.triangleVertexes[2].gl_Position.y;
        float a3 = triangle.triangleVertexes[2].gl_Position.y - triangle.triangleVertexes[0].gl_Position.y;

        float b1 = triangle.triangleVertexes[1].gl_Position.x - triangle.triangleVertexes[0].gl_Position.x;
        float b2 = triangle.triangleVertexes[2].gl_Position.x - triangle.triangleVertexes[1].gl_Position.x;
        float b3 = triangle.triangleVertexes[0].gl_Position.x - triangle.triangleVertexes[2].gl_Position.x;
        //c for general line equation
        float c1 = (-1.f * (a1 * triangle.triangleVertexes[0].gl_Position.x + b1 * triangle.triangleVertexes[0].gl_Position.y));
        float c2 = (-1.f * (a2 * triangle.triangleVertexes[1].gl_Position.x + b2 * triangle.triangleVertexes[1].gl_Position.y));
        float c3 = (-1.f * (a3 * triangle.triangleVertexes[2].gl_Position.x + b3 * triangle.triangleVertexes[2].gl_Position.y));

        float edge1_prime = a1 * (minX + 0.5f) + b1 * (minY + 0.5f) + c1;
        float edge2_prime = a2 * (minX + 0.5f) + b2 * (minY + 0.5f) + c2;
        float edge3_prime = a3 * (minX + 0.5f) + b3 * (minY + 0.5f) + c3;
        float edge1_edited, edge2_edited, edge3_edited;

        for (int y = minY; y <= maxY; y++)
        {
            edge1_edited = edge1_prime;
            edge2_edited = edge2_prime;
            edge3_edited = edge3_prime;

            for (int x = minX; x <= maxX; x++)
            {
                if (edge1_edited >= 0 && edge2_edited >= 0 && edge3_edited >= 0)
                {

                    InFragment fragment = assembleInFragment(x, y, triangle);
                    OutFragment color = fragmentProcessor(fragment);
                    perFragment(x, y, fragment.gl_FragCoord.z, color);
                }
                else
                {
                    int boga = x;
                    int boga2 = y;
                    int lukasek = 54;
                }
                edge1_edited += a1;
                edge2_edited += a2;
                edge3_edited += a3;
            }
            edge1_prime += b1;
            edge2_prime += b2;
            edge3_prime += b3;
        }
    }
}



/// @}