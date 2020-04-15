/*!
 * @file
 * @brief This file contains implementation of phong rendering method
 *
 * @author Tomáš Milet, imilet@fit.vutbr.cz
 */

#include <student/phongMethod.hpp>
#include <student/bunny.hpp>

/** \addtogroup shader_side 06. Implementace vertex/fragment shaderu phongovy metody
 * Vašim úkolem ve vertex a fragment shaderu je transformovat trojúhelníky pomocí view a projekční matice a spočítat phongův osvětlovací model.
 * Vašim úkolem také je správně vypočítat procedurální barvu.
 * Následující obrázek zobrazuje shadery v různém stupni implementace.
 * Horní řáděk zobrazuje procedurální texturu.
 * Prostřední řádek zobrazuje model králička s nanesenou texturou, ale bez použití transformačních matic.
 * Dolní řádek zobrazuje model po použítí transformačních matic.
 *
 * \image html images/texture.svg "Vypočet procedurální textury." width=1000
 *
 * 
 *
 *
 *
 * @{
 */


/**
 * @brief This function represents vertex shader of phong method.
 *
 * @param outVertex output vertex
 * @param inVertex input vertex
 * @param uniforms uniform variables
 */
void phong_VS(OutVertex&outVertex,InVertex const&inVertex,Uniforms const&uniforms){


    //position in world space
    outVertex.attributes[0] = inVertex.attributes[0];
    //normal vector in world space
    outVertex.attributes[1] = inVertex.attributes[1];
    //P*V;
    glm::mat4 projectionView = uniforms.uniform[1].m4 * uniforms.uniform[0].m4;
        //multiply by view matrix
    glm::vec4 position_vector = glm::vec4(inVertex.attributes[0].v3,1.f);
    outVertex.gl_Position = projectionView * position_vector;
}

/**
 * @brief This function represents fragment shader of phong method.
 *
 * @param outFragment output fragment
 * @param inFragment input fragment
 * @param uniforms uniform variables
 */
void phong_FS(OutFragment&outFragment,InFragment const&inFragment,Uniforms const&uniforms){

    glm::vec4 color;

    
    float x = inFragment.attributes[0].v2[0];
    float y = inFragment.attributes[0].v2[1];
    float whole = (x + (sinf(y * 10) / 10)) * 5;

    float decimal = whole - floor(whole);
    //green and yellow stripes
    color = decimal > 0.5f ? glm::vec4(1, 1, 0, 0) : glm::vec4(0, 0.5f, 0, 0);
    float normalY = inFragment.attributes[1].v2[1];
    //snow texture interpolation
    if (normalY > 0.f)
    {
        float t = normalY * normalY;
        color = (1 - t) * color + t * glm::vec4(1.f, 1.f, 1.f, 0);
    }
    //light
    //bod - pozice svetla
    //vysledek vec3 * normala vec3
    //vysledek vynasobit color

   glm::vec3 lightVectorNormalized = glm::normalize(uniforms.uniform[2].v3 - inFragment.attributes[0].v3);
   glm::vec3 normalVectorNormalized = glm::normalize(inFragment.attributes[1].v3);
   glm::vec3 cameraVectorNormalized = glm::normalize(uniforms.uniform[3].v3 - inFragment.attributes[0].v3);
   glm::vec3 rotatedLightVector = glm::reflect(cameraVectorNormalized, normalVectorNormalized);
   float cosLightNormal = glm::dot(lightVectorNormalized, normalVectorNormalized);
   float cosCamLight = glm::dot(rotatedLightVector, lightVectorNormalized);

    cosLightNormal = cosLightNormal > 0 ? cosLightNormal : 0;
    cosCamLight = cosCamLight < 0 ? cosCamLight: 0;
    cosCamLight = std::pow(cosCamLight, 40.f);
    color = glm::vec4(color.r * cosLightNormal , color.g * cosLightNormal, color.b * cosLightNormal, color.a);
    color = glm::vec4(color.r + cosCamLight, color.g + cosCamLight, color.b + cosCamLight, color.a);
    color = glm::clamp(color,0.f,1.f);
    outFragment.gl_FragColor = color;


}

/// @}

/** \addtogroup cpu_side 07. Implementace vykreslení králička s phongovým osvětlovacím modelem.
 * Vaším úkolem je využít naimplementovanou grafickou kartu a vykreslit králička s phongovým osvětlovacím modelem a stínováním a procedurální texturou.
 * @{
 */


/**
 * @brief Constructoro f phong method
 */
PhongMethod::PhongMethod(){



    vp = gpu.createVertexPuller();
    //setup indexing buffer
    auto indicesSize = sizeof(bunnyIndices);
    indexingBuffer = gpu.createBuffer(indicesSize);
    gpu.setBufferData(indexingBuffer,0,indicesSize, (void*)bunnyIndices);
    gpu.setVertexPullerIndexing(vp, IndexType::UINT32,indexingBuffer);


    //set attributes data and vertex puller's heads
    auto verticesSize = sizeof(bunnyVertices);
    verticesBuffer = gpu.createBuffer(verticesSize);
    gpu.setBufferData(verticesBuffer, 0,verticesSize,(void*)bunnyVertices);
  
    gpu.setVertexPullerHead(vp, 0, AttributeType::VEC3, sizeof(float) * 6, 0, verticesBuffer);//remove vectors data
    gpu.setVertexPullerHead(vp, 1, AttributeType::VEC3, sizeof(float) * 6, 3 * sizeof(float), verticesBuffer);//remove vectors data  
    gpu.enableVertexPullerHead(vp,0);
    gpu.enableVertexPullerHead(vp, 1);
    //create program and attach shaders + attribute types(attributes = normal vectors and 
    shaderProgram = gpu.createProgram();
    gpu.attachShaders(shaderProgram, phong_VS, phong_FS);
    gpu.setVS2FSType(shaderProgram, 0, AttributeType::VEC3);
    gpu.setVS2FSType(shaderProgram, 1, AttributeType::VEC3);
    

}


/**
 * @brief This function draws phong method.
 *
 * @param proj projection matrix
 * @param view view matrix
 * @param light light position
 * @param camera camera position
 */
void PhongMethod::onDraw(glm::mat4 const&proj,glm::mat4 const&view,glm::vec3 const&light,glm::vec3 const&camera){


/// <b>Seznam funkcí, které jistě využijete:</b>
///  - gpu.bindVertexPuller()
///  - gpu.useProgram()
///  - gpu.programUniformMatrix4f()
///  - gpu.programUniform3f      ()
///  - gpu.drawTriangles()
///  - gpu.unbindVertexPuller()

    //nastavit aktivni program a vertex puller
    //data z parametru fo uniforms
    //zavolat draw triangles (kolik ??)


    gpu.clear(.5f, .5f, .5f, 1.f);

    gpu.bindVertexPuller(vp);
    gpu.useProgram(shaderProgram);
    gpu.programUniformMatrix4f(shaderProgram, 0, view);
    gpu.programUniformMatrix4f(shaderProgram, 1, proj);
    gpu.programUniform3f(shaderProgram, 2, light);
    gpu.programUniform3f(shaderProgram, 3, camera);
    gpu.drawTriangles(sizeof(bunnyIndices) / (sizeof(VertexIndex))); 

    gpu.unbindVertexPuller();

   
    

 

    



}

/**
 * @brief Destructor of phong method.
 */
PhongMethod::~PhongMethod(){
  ///\todo Zde uvolněte alokované zdroje
  /// <b>Seznam funkcí</b>
  ///  - gpu.deleteProgram()
  ///  - gpu.deleteVertexPuller()
  ///  - gpu.deleteBuffer()
    gpu.deleteProgram(shaderProgram);
    gpu.deleteVertexPuller(vp);
    gpu.deleteBuffer(indexingBuffer);
    gpu.deleteBuffer(verticesBuffer);

}

/// @}
