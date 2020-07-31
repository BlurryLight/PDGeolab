#version 330 core

out vec4 FragColor;
in float curv;
in VS_OUT
{
    vec3 Normal;
    vec2 TexCoords;
    vec3 WorldPos;
} fs_in;

uniform vec3 viewPos = vec3(0.0);
uniform float maxg;
uniform float ming;
vec4 BlinnPhongCalc();
void main()
{
    float g = (curv - ming) / (maxg - ming);
    FragColor = vec4(1.0f,g,0.0f,1.0f);
}
vec4 BlinnPhongCalc()
{
    float r = distance(vec3(0.0),fs_in.WorldPos);
    float attenuation = 1.0 / (1.0 + 0.1 * r );
    //ambient
    vec3 res = 0.3 * vec3(0.5);
    //diffuse
    vec3 lightDir = normalize(vec3(0.0) - fs_in.WorldPos);
    vec3 normal = normalize(fs_in.Normal);
    float LdotN = max(dot(lightDir,normal),0.0);
    res += LdotN *  vec3(0.5) * attenuation;

    //specular
    vec3 viewDir = normalize(viewPos - fs_in.WorldPos);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float HdotN = max(dot(normal,halfwayDir),0.0);
    float spec_index = 0.0;
    if(HdotN > 0.0)
    {
        spec_index = pow(HdotN, 32);
    }
    res+= spec_index * vec3(1.0) * attenuation;
    return vec4(res,1.0);
}
