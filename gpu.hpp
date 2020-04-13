/*!
 * @file
 * @brief This file contains class that represents graphic card.
 *
 * @author Tomáš Milet, imilet@fit.vutbr.cz
 */
#pragma once

#include <student/fwd.hpp>
#include <vector>
#include <map>
#include<array>

/**
 * @brief This class represent software GPU
 */
class GPU{
  public:
    GPU();
    virtual ~GPU();

    //buffer object commands
    BufferID  createBuffer           (uint64_t size);
    void      deleteBuffer           (BufferID buffer);
    void      setBufferData          (BufferID buffer,uint64_t offset,uint64_t size,void const* data);
    void      getBufferData          (BufferID buffer,uint64_t offset,uint64_t size,void      * data);
    bool      isBuffer               (BufferID buffer);

    //vertex array object commands (vertex puller)
    ObjectID  createVertexPuller     ();
    void      deleteVertexPuller     (VertexPullerID vao);
    void      setVertexPullerHead    (VertexPullerID vao,uint32_t head,AttributeType type,uint64_t stride,uint64_t offset,BufferID buffer);
    void      setVertexPullerIndexing(VertexPullerID vao,IndexType type,BufferID buffer);
    void      enableVertexPullerHead (VertexPullerID vao,uint32_t head);
    void      disableVertexPullerHead(VertexPullerID vao,uint32_t head);
    void      bindVertexPuller       (VertexPullerID vao);
    void      unbindVertexPuller     ();
    bool      isVertexPuller         (VertexPullerID vao);

    //program object commands
    ProgramID createProgram          ();
    void      deleteProgram          (ProgramID prg);
    void      attachShaders          (ProgramID prg,VertexShader vs,FragmentShader fs);
    void      setVS2FSType           (ProgramID prg,uint32_t attrib,AttributeType type);
    void      useProgram             (ProgramID prg);
    bool      isProgram              (ProgramID prg);
    void      programUniform1f       (ProgramID prg,uint32_t uniformId,float     const&d);
    void      programUniform2f       (ProgramID prg,uint32_t uniformId,glm::vec2 const&d);
    void      programUniform3f       (ProgramID prg,uint32_t uniformId,glm::vec3 const&d);
    void      programUniform4f       (ProgramID prg,uint32_t uniformId,glm::vec4 const&d);
    void      programUniformMatrix4f (ProgramID prg,uint32_t uniformId,glm::mat4 const&d);

    //framebuffer functions
    void      createFramebuffer      (uint32_t width,uint32_t height);
    void      deleteFramebuffer      ();
    void      resizeFramebuffer      (uint32_t width,uint32_t height);
    uint8_t*  getFramebufferColor    ();
    float*    getFramebufferDepth    ();
    uint32_t  getFramebufferWidth    ();
    uint32_t  getFramebufferHeight   ();

    //execution commands
    void      clear                  (float r,float g,float b,float a);
    void      drawTriangles          (uint32_t  nofVertices);

    /// \addtogroup gpu_init 00. proměnné, inicializace / deinicializace grafické karty
    /// @{
    /// \todo zde si můžete vytvořit proměnné grafické karty (buffery, programy, ...)
    /// @}
private:
   
    //buffer
    struct Buffer
    {
       // BufferID ID;
        uint8_t* data;
        uint64_t size;
    };
   // std::vector<Buffer*> buffers;
    std::map<BufferID, Buffer*> buffers;
    //vertexPuller
    struct Head
    {
        BufferID ID = 0;
        uint64_t offset = 0;
        uint64_t stride = 0;
        AttributeType attrType = AttributeType::EMPTY;
        bool enabled = false;
    };
    struct Indexing
    {
        BufferID bufferID = 0; //null if not set - buffers use address as ID, 0 is invalid
        IndexType indexType;
    };
    struct VertexPuller
    {
        Indexing indexing;
        std::array<Head, maxAttributes> heads; 
    };
    std::map<VertexPullerID, VertexPuller*> vertexPullers;
    VertexPullerID activeVertexPullerID;

    //shader program
    struct ShaderProgram
    {
        VertexShader vertexShader = NULL;
        FragmentShader fragmentShader = NULL;
        Uniforms uniforms;
        std::array<AttributeType, maxAttributes> attributes{ AttributeType::EMPTY };
    };
    std::map<ProgramID, ShaderProgram*> programs;
    ProgramID activeProgramID = NULL;

    struct Color
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    };
    struct FrameBuffer
    {
        uint32_t width = 0;
        uint32_t height = 0;
        std::vector<Color>colorBuffer;
        std::vector<float>depthBuffer;
    };
    FrameBuffer frameBuffer;
    struct Triangle
    {
        std::array<OutVertex, 3> triangleVertexes;
    };

    //returns vector of triangle structures - assembled 3 vertexes
    std::vector<Triangle> assembleTriangles(uint32_t trianglesCount);
    //makes OutVertex from InVertex by execution of vertexShader
    OutVertex vertexProcessor(uint32_t vertexNumber);
    //assembles invertex from buffers
    InVertex vertexPullerRead(uint32_t vertexNumber);
    //clips all triangles from given vector
    //vector might contain more triangles than before execution (cause by clipping only 1 vertex in triangle)
    std::vector<Triangle> clipTriangles(std::vector<Triangle> triangles);
    //return true, if vertex is in view space - no need for clipping
    bool inViewSpace(OutVertex vertex);
    //cuts edge AB and returns new vertex B to replace clipped vertex
    OutVertex cutEdge(OutVertex vertexInside, OutVertex vertexOutside);
    //normalize coords to -1;+1 range, transform coords to frame buffer width, height
    void viewportTransAndNormalize(std::vector<Triangle>& triangles);
    //rasterization algorithm
    void rasterize(std::vector<Triangle> triangles);
    //executes fragment shader function, returns Outfragment structure containing RGBA color
    OutFragment fragmentProcessor(InFragment fragment);
    //tests if pixel should be replaced - depth is lower than current pixel's depth
    bool depthTest(uint32_t x, uint32_t y, float depth);
    //overwrites values in framebuffer if depthtest == true
    void GPU::perFragment(uint32_t x, uint32_t y, float depth, OutFragment color);
    //returns assembled InFragment structure with normalized coordinates and interpolated attributes and depth
    InFragment GPU::assembleInFragment(uint64_t x, uint64_t y, GPU::Triangle triangle);
    //computes array of triangle from 3 vertexes with x and y float coords
    float GPU::triangleArea(std::array<std::array<float, 2>, 3> edgeVectors);
};


