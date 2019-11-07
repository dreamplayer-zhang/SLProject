#include <AverageTiming.h>
#include <ImageProcessor.h>
#include <SLSceneView.h>
#include <SLPoints.h>
#include <SLPolyline.h>
#include <CVCalibration.h>
//#include <BRIEFpattern.h>

#define INPUT 0
#define D2GDX2 1
#define D2GDY2 2
#define DGDX 3
#define GXX 4
#define GYY 5
#define GXY 6
#define DETH 7
#define NMSX 8
#define NMSY 9
#define NMSZ 10
#define EXTRACTOR 11

static std::string textureOfstFct = "\n"
                                    "vec3 Ix(float ofst)\n"
                                    "{\n"
                                    "    return texture(tex, texcoords + vec2(ofst, 0.0)).rgb;\n"
                                    "}\n"
                                    "\n"
                                    "vec3 Iy(float ofst)\n"
                                    "{\n"
                                    "    return texture(tex, texcoords + vec2(0.0, ofst)).rgb;\n"
                                    "}\n"
                                    "\n";

static std::string normalizedFct = "\n"
                                   "vec3 UN(vec3 v)\n"
                                   "{\n"
                                   "    return  v;\n"
                                   "}\n"
                                   "\n"
                                   "vec3 N(vec3 v)\n"
                                   "{\n"
                                   "    return v;\n"
                                   "}\n"
                                   "\n"
                                   "float UN(float v)\n"
                                   "{\n"
                                   "    return v;\n"
                                   "}\n"
                                   "\n"
                                   "float N(float v)\n"
                                   "{\n"
                                   "    return v;\n"
                                   "}\n";

/*

#define MUL "1.0"
#define DIV "1.0"
#define OFT "0.0"

static std::string normalizedFct = "\n"
                                   "vec3 UN(vec3 v)\n"
                                   "{\n"
                                   "    return -vec3(" OFT ") + " MUL " * v;\n"
                                   "}\n"
                                   "\n"
                                   "vec3 N(vec3 v)\n"
                                   "{\n"
                                   "    return " DIV " * (v + vec3(" OFT "));\n"
                                   "}\n"
                                   "\n"
                                   "float UN(float v)\n"
                                   "{\n"
                                   "    return -" OFT " + " MUL " * v;\n"
                                   "}\n"
                                   "\n"
                                   "float N(float v)\n"
                                   "{\n"
                                   "    return " DIV " * (v + " OFT ");\n"
                                   "}\n";
*/

//#version 320 es
static std::string screeQuadVs = "layout (location = 0) in vec3 vcoords;\n"
                                 "out vec2 texcoords;\n"
                                 "\n"
                                 "void main()\n"
                                 "{\n"
                                 "    texcoords = 0.5 * (vcoords.xy + vec2(1.0));\n"
                                 "    gl_Position = vec4(vcoords, 1.0);\n"
                                 "}\n";

static std::string hGaussianFs = "#ifdef GL_ES\n"
                                 "precision highp float;\n"
                                 "#endif\n"
                                 "out vec3 pixel;\n"
                                 "in vec2 texcoords;\n"
                                 "uniform float w;\n"
                                 "uniform sampler2D tex;\n"
                                 "\n"
                                 "#include texOffsets\n"
                                 "#include normalizer\n"
                                 "\n"
                                 "const float kernel12[9] = float[9]("
                                 "0.007614419169296346, 0.03607496968918392, 0.10958608179781393, 0.2134445419434044, 0.26655997480060273, 0.21344454194340445, 0.109586081797814, 0.036074969689183944, 0.007614419169296356);\n"
                                 "\n"
                                 "const float kernel20[15] = float[15]("
                                 "0.0031742033144480037, 0.008980510024247402, 0.02165110898093487, 0.04448075733770272, 0.07787123866346017, 0.11617023707406768, 0.1476813151730447, 0.15998125886418896, 0.14768131517304472, 0.11617023707406769, 0.07787123866346018, 0.04448075733770272, 0.02165110898093487, 0.008980510024247402, 0.0031742033144480037);\n"
                                 "\n"
                                 "const float kernel28[21] = float[21]("
                                 "0.0019290645132252363, 0.004189349123089384, 0.008384820035189703, 0.015466367540072906, 0.02629240397422038, 0.041192642776781994, 0.05947800651444567, 0.07914810874862578, 0.09706710312973113, 0.10971120494447856, 0.11428185740027867, 0.10971120494447856, 0.09706710312973113, 0.07914810874862578, 0.05947800651444567, 0.041192642776781994, 0.02629240397422038, 0.015466367540072906, 0.008384820035189703, 0.004189349123089384, 0.0019290645132252363);\n"
                                 "\n"
                                 "void main()\n"
                                 "{\n"
                                 "    \n"
                                 "    vec3 response = vec3(0.0);\n"
                                 "    for (int i = 0; i < 3; i++)\n"
                                 "    {\n"
                                 "        response.b += kernel28[i] * UN(Ix((float(i) - 10.0) / w).b);\n"
                                 "    }\n"
                                 "    for (int i = 0; i < 3; i++)\n"
                                 "    {\n"
                                 "        vec3 v = UN(Ix((float(i) - 7.0) / w));\n"
                                 "        response.b += kernel28[i+3] * v.b;\n"
                                 "        response.g += kernel20[i  ] * v.g;\n"
                                 "    }\n"
                                 "    for (int i = 0; i < 9; i++)\n"
                                 "    {\n"
                                 "        vec3 v = UN(Ix((float(i) - 4.0) / w));\n"
                                 "        response.b += kernel28[i+6] * v.b;\n"
                                 "        response.g += kernel20[i+3] * v.g;\n"
                                 "        response.r += kernel12[i  ] * v.r;\n"
                                 "    }\n"
                                 "    for (int i = 12; i < 15; i++)\n"
                                 "    {\n"
                                 "        vec3 v = UN(Ix((float(i) - 7.0) / w));\n"
                                 "        response.g += kernel20[i  ] * v.g;\n"
                                 "        response.b += kernel28[i+3] * v.b;\n"
                                 "    }\n"
                                 "    for (int i = 18; i < 21; i++)\n"
                                 "    {\n"
                                 "        response.b += kernel28[i] * UN(Ix((float(i) - 10.0) / w).b);\n"
                                 "    }\n"
                                 "    pixel = N(response);\n"
                                 "}\n";

static std::string vGaussianFs = "#ifdef GL_ES\n"
                                 "precision highp float;\n"
                                 "#endif\n"
                                 "out vec3 pixel;\n"
                                 "in vec2 texcoords;\n"
                                 "uniform float w;\n"
                                 "uniform sampler2D tex;\n"
                                 "\n"
                                 "#include texOffsets\n"
                                 "#include normalizer\n"
                                 "\n"
                                 "const float kernel12[9] = float[9]("
                                 "0.007614419169296346, 0.03607496968918392, 0.10958608179781393, 0.2134445419434044, 0.26655997480060273, 0.21344454194340445, 0.109586081797814, 0.036074969689183944, 0.007614419169296356);\n"
                                 "\n"
                                 "const float kernel20[15] = float[15]("
                                 "0.0031742033144480037, 0.008980510024247402, 0.02165110898093487, 0.04448075733770272, 0.07787123866346017, 0.11617023707406768, 0.1476813151730447, 0.15998125886418896, 0.14768131517304472, 0.11617023707406769, 0.07787123866346018, 0.04448075733770272, 0.02165110898093487, 0.008980510024247402, 0.0031742033144480037);\n"
                                 "\n"
                                 "const float kernel28[21] = float[21]("
                                 "0.0019290645132252363, 0.004189349123089384, 0.008384820035189703, 0.015466367540072906, 0.02629240397422038, 0.041192642776781994, 0.05947800651444567, 0.07914810874862578, 0.09706710312973113, 0.10971120494447856, 0.11428185740027867, 0.10971120494447856, 0.09706710312973113, 0.07914810874862578, 0.05947800651444567, 0.041192642776781994, 0.02629240397422038, 0.015466367540072906, 0.008384820035189703, 0.004189349123089384, 0.0019290645132252363);\n"
                                 "\n"
                                 "void main()\n"
                                 "{\n"
                                 "\n"
                                 "\n"
                                 "    \n"
                                 "    vec3 response = vec3(0.0);\n"
                                 "    for (int i = 0; i < 3; i++)\n"
                                 "    {\n"
                                 "        response.b += kernel28[i] * UN(Iy((float(i) - 10.0) / w).b);\n"
                                 "    }\n"
                                 "    for (int i = 0; i < 3; i++)\n"
                                 "    {\n"
                                 "        vec3 v = UN(Iy((float(i) - 7.0) / w));\n"
                                 "        response.b += kernel28[i+3] * v.b;\n"
                                 "        response.g += kernel20[i  ] * v.g;\n"
                                 "    }\n"
                                 "    for (int i = 0; i < 9; i++)\n"
                                 "    {\n"
                                 "        vec3 v = UN(Iy((float(i) - 4.0) / w));\n"
                                 "        response.b += kernel28[i+6] * v.b;\n"
                                 "        response.g += kernel20[i+3] * v.g;\n"
                                 "        response.r += kernel12[i  ] * v.r;\n"
                                 "    }\n"
                                 "    for (int i = 12; i < 15; i++)\n"
                                 "    {\n"
                                 "        vec3 v = UN(Iy((float(i) - 7.0) / w));\n"
                                 "        response.g += kernel20[i  ] * v.g;\n"
                                 "        response.b += kernel28[i+3] * v.b;\n"
                                 "    }\n"
                                 "    for (int i = 18; i < 21; i++)\n"
                                 "    {\n"
                                 "        response.b += kernel28[i] * UN(Iy((float(i) - 10.0) / w).b);\n"
                                 "    }\n"
                                 "    pixel = N(response);\n"
                                 "}\n";

static std::string hGaussianDxFs = "#ifdef GL_ES\n"
                                   "precision highp float;\n"
                                   "#endif\n"
                                   "out vec3 pixel;\n"
                                   "in vec2 texcoords;\n"
                                   "uniform float w;\n"
                                   "uniform sampler2D tex;\n"
                                   "\n"
                                   "#include texOffsets\n"
                                   "#include normalizer\n"
                                   "\n"
                                   "const float kernel12[9] = float[9]("
                                   "0.02030511778479025, 0.07214993937836783, 0.14611477573041856, 0.14229636129560294, 0.0, -0.1422963612956029, -0.14611477573041862, -0.07214993937836786, -0.02030511778479028);\n"
                                   "\n"
                                   "const float kernel20[15] = float[15]("
                                   "0.00888776928045441, 0.021553224058193758, 0.04330221796186974, 0.07116921174032437, 0.09344548639615223, 0.09293618965925417, 0.0590725260692179, 0.0, -0.059072526069217875, -0.09293618965925415, -0.09344548639615223, -0.07116921174032437, -0.04330221796186974, -0.021553224058193758, -0.00888776928045441);\n"
                                   "\n"
                                   "const float kernel28[21] = float[21]("
                                   "0.005511612894929245, 0.01077261203080127, 0.01916530293757646, 0.03093273508014581, 0.04507269252723493, 0.058846632538259974, 0.06797486458793789, 0.06784123607025065, 0.05546691607413207, 0.0313460585555653, -0.0, -0.0313460585555653, -0.05546691607413207, -0.06784123607025065, -0.06797486458793789, -0.058846632538259974, -0.04507269252723493, -0.03093273508014581, -0.01916530293757646, -0.01077261203080127, -0.005511612894929245);\n"
                                   "\n"
                                   "void main()\n"
                                   "{\n"
                                   "\n"
                                   "\n"
                                   "    \n"
                                   "    vec3 response = vec3(0.0);\n"
                                   "    for (int i = 0; i < 3; i++)\n"
                                   "    {\n"
                                   "        response.b += kernel28[i] * Ix((float(i) - 10.0) / w).r;\n"
                                   "    }\n"
                                   "    for (int i = 0; i < 3; i++)\n"
                                   "    {\n"
                                   "        float v = Ix((float(i) - 7.0) / w).r;\n"
                                   "        response.b += kernel28[i+3] * v;\n"
                                   "        response.g += kernel20[i  ] * v;\n"
                                   "    }\n"
                                   "    for (int i = 0; i < 9; i++)\n"
                                   "    {\n"
                                   "        float v = Ix((float(i) - 4.0) / w).r;\n"
                                   "        response.b += kernel28[i+6] * v;\n"
                                   "        response.g += kernel20[i+3] * v;\n"
                                   "        response.r += kernel12[i  ] * v;\n"
                                   "    }\n"
                                   "    for (int i = 12; i < 15; i++)\n"
                                   "    {\n"
                                   "        float v = Ix((float(i) - 7.0) / w).r;\n"
                                   "        response.g += kernel20[i  ] * v;\n"
                                   "        response.b += kernel28[i+3] * v;\n"
                                   "    }\n"
                                   "    for (int i = 18; i < 21; i++)\n"
                                   "    {\n"
                                   "        float ofst = (float(i) - 10.0) / w;\n"
                                   "        response.b += kernel28[i] * Ix((float(i) - 10.0) / w).r;\n"
                                   "    }\n"
                                   "    pixel = N(response);\n"
                                   "}\n";

static std::string vGaussianDyFs = "#ifdef GL_ES\n"
                                   "precision highp float;\n"
                                   "#endif\n"
                                   "out vec3 pixel;\n"
                                   "in vec2 texcoords;\n"
                                   "uniform float w;\n"
                                   "uniform sampler2D tex;\n"
                                   "\n"
                                   "#include texOffsets\n"
                                   "#include normalizer\n"
                                   "\n"
                                   "const float kernel12[9] = float[9]("
                                   "0.02030511778479025, 0.07214993937836783, 0.14611477573041856, 0.14229636129560294, 0.0, -0.1422963612956029, -0.14611477573041862, -0.07214993937836786, -0.02030511778479028);\n"
                                   "\n"
                                   "const float kernel20[15] = float[15]("
                                   "0.00888776928045441, 0.021553224058193758, 0.04330221796186974, 0.07116921174032437, 0.09344548639615223, 0.09293618965925417, 0.0590725260692179, 0.0, -0.059072526069217875, -0.09293618965925415, -0.09344548639615223, -0.07116921174032437, -0.04330221796186974, -0.021553224058193758, -0.00888776928045441);\n"
                                   "\n"
                                   "const float kernel28[21] = float[21]("
                                   "0.005511612894929245, 0.01077261203080127, 0.01916530293757646, 0.03093273508014581, 0.04507269252723493, 0.058846632538259974, 0.06797486458793789, 0.06784123607025065, 0.05546691607413207, 0.0313460585555653, -0.0, -0.0313460585555653, -0.05546691607413207, -0.06784123607025065, -0.06797486458793789, -0.058846632538259974, -0.04507269252723493, -0.03093273508014581, -0.01916530293757646, -0.01077261203080127, -0.005511612894929245);\n"
                                   "\n"
                                   "void main()\n"
                                   "{\n"
                                   "    \n"
                                   "    vec3 response = vec3(0.0);\n"
                                   "    for (int i = 0; i < 3; i++)\n"
                                   "    {\n"
                                   "        response.b += kernel28[i] * UN(Iy((float(i) - 10.0) / w).b);\n"
                                   "    }\n"
                                   "    for (int i = 0; i < 3; i++)\n"
                                   "    {\n"
                                   "        vec3 v = UN(Iy((float(i) - 7.0) / w));\n"
                                   "        response.b += kernel28[i+3] * v.b;\n"
                                   "        response.g += kernel20[i  ] * v.g;\n"
                                   "    }\n"
                                   "    for (int i = 0; i < 9; i++)\n"
                                   "    {\n"
                                   "        vec3 v = UN(Iy((float(i) - 4.0) / w));\n"
                                   "        response.b += kernel28[i+6] * v.b;\n"
                                   "        response.g += kernel20[i+3] * v.g;\n"
                                   "        response.r += kernel12[i  ] * v.r;\n"
                                   "    }\n"
                                   "    for (int i = 12; i < 15; i++)\n"
                                   "    {\n"
                                   "        vec3 v = UN(Iy((float(i) - 7.0) / w));\n"
                                   "        response.g += kernel20[i  ] * v.g;\n"
                                   "        response.b += kernel28[i+3] * v.b;\n"
                                   "    }\n"
                                   "    for (int i = 18; i < 21; i++)\n"
                                   "    {\n"
                                   "        response.b += kernel28[i] * UN(Iy((float(i) - 10.0) / w).b);\n"
                                   "    }\n"
                                   "    pixel = N(response);\n"
                                   "}\n";

static std::string hGaussianDx2Fs = "#ifdef GL_ES\n"
                                    "precision highp float;\n"
                                    "#endif\n"
                                    "out vec3 pixel;\n"
                                    "in vec2 texcoords;\n"
                                    "uniform float w;\n"
                                    "uniform sampler2D tex;\n"
                                    "\n"
                                    "#include texOffsets\n"
                                    "#include normalizer\n"
                                    "\n"
                                    "const float kernel12[9] = float[9]("
                                    "0.04653256159014432, 0.10822490906755174, 0.08523361917607754, -0.11858030107966908, -0.2665599748006027, -0.1185803010796692, 0.08523361917607745, 0.10822490906755174, 0.046532561590144364);\n"
                                    "\n"
                                    "const float kernel20[15] = float[15]("
                                    "0.021711550670824344, 0.04274722771541763, 0.0649533269428046, 0.06938998144681627, 0.034263345011922505, -0.04182128534666434, -0.12405230474535753, -0.15998125886418896, -0.12405230474535757, -0.041821285346664384, 0.03426334501192248, 0.06938998144681627, 0.0649533269428046, 0.04274722771541763, 0.021711550670824344);\n"
                                    "\n"
                                    "const float kernel28[21] = float[21]("
                                    "0.013818400900858318, 0.02351165324182817, 0.035421586679270776, 0.046399102620218693, 0.05097506892961091, 0.04287397513501796, 0.018207553014626197, -0.020998477831268087, -0.06537172251594138, -0.10075518821431705, -0.11428185740027866, -0.10075518821431705, -0.06537172251594138, -0.020998477831268087, 0.018207553014626197, 0.04287397513501796, 0.05097506892961091, 0.046399102620218693, 0.035421586679270776, 0.02351165324182817, 0.013818400900858318);\n"
                                    "\n"
                                    "void main()\n"
                                    "{\n"
                                    "    \n"
                                    "    vec3 response = vec3(0.0);\n"
                                    "    for (int i = 0; i < 3; i++)\n"
                                    "    {\n"
                                    "        response.b += kernel28[i] * Ix((float(i) - 10.0) / w).r;\n"
                                    "    }\n"
                                    "    for (int i = 0; i < 3; i++)\n"
                                    "    {\n"
                                    "        float v = Ix((float(i) - 7.0) / w).r;\n"
                                    "        response.b += kernel28[i+3] * v;\n"
                                    "        response.g += kernel20[i  ] * v;\n"
                                    "    }\n"
                                    "    for (int i = 0; i < 9; i++)\n"
                                    "    {\n"
                                    "        float v = Ix((float(i) - 4.0) / w).r;\n"
                                    "        response.b += kernel28[i+6] * v;\n"
                                    "        response.g += kernel20[i+3] * v;\n"
                                    "        response.r += kernel12[i  ] * v;\n"
                                    "    }\n"
                                    "    for (int i = 12; i < 15; i++)\n"
                                    "    {\n"
                                    "        float v = Ix((float(i) - 7.0) / w).r;\n"
                                    "        response.g += kernel20[i  ] * v;\n"
                                    "        response.b += kernel28[i+3] * v;\n"
                                    "    }\n"
                                    "    for (int i = 18; i < 21; i++)\n"
                                    "    {\n"
                                    "        response.b += kernel28[i] * Ix((float(i) - 10.0) / w).r;\n"
                                    "    }\n"
                                    "    pixel = N(response);\n"
                                    "}\n";

static std::string vGaussianDy2Fs = "#ifdef GL_ES\n"
                                    "precision highp float;\n"
                                    "#endif\n"
                                    "out vec3 pixel;\n"
                                    "in vec2 texcoords;\n"
                                    "uniform float w;\n"
                                    "uniform sampler2D tex;\n"
                                    "\n"
                                    "#include texOffsets\n"
                                    "#include normalizer\n"
                                    "\n"
                                    "const float kernel12[9] = float[9]("
                                    "0.04653256159014432, 0.10822490906755174, 0.08523361917607754, -0.11858030107966908, -0.2665599748006027, -0.1185803010796692, 0.08523361917607745, 0.10822490906755174, 0.046532561590144364);\n"
                                    "\n"
                                    "const float kernel20[15] = float[15]("
                                    "0.021711550670824344, 0.04274722771541763, 0.0649533269428046, 0.06938998144681627, 0.034263345011922505, -0.04182128534666434, -0.12405230474535753, -0.15998125886418896, -0.12405230474535757, -0.041821285346664384, 0.03426334501192248, 0.06938998144681627, 0.0649533269428046, 0.04274722771541763, 0.021711550670824344);\n"
                                    "\n"
                                    "const float kernel28[21] = float[21]("
                                    "0.013818400900858318, 0.02351165324182817, 0.035421586679270776, 0.046399102620218693, 0.05097506892961091, 0.04287397513501796, 0.018207553014626197, -0.020998477831268087, -0.06537172251594138, -0.10075518821431705, -0.11428185740027866, -0.10075518821431705, -0.06537172251594138, -0.020998477831268087, 0.018207553014626197, 0.04287397513501796, 0.05097506892961091, 0.046399102620218693, 0.035421586679270776, 0.02351165324182817, 0.013818400900858318);\n"
                                    "\n"
                                    "void main()\n"
                                    "{\n"
                                    "    \n"
                                    "    vec3 response = vec3(0.0);\n"
                                    "    for (int i = 0; i < 3; i++)\n"
                                    "    {\n"
                                    "        response.b += kernel28[i] * Iy((float(i) - 10.0) / w).r;\n"
                                    "    }\n"
                                    "    for (int i = 0; i < 3; i++)\n"
                                    "    {\n"
                                    "        float v = Iy((float(i) - 7.0) / w).r;\n"
                                    "        response.b += kernel28[i+3] * v;\n"
                                    "        response.g += kernel20[i  ] * v;\n"
                                    "    }\n"
                                    "    for (int i = 0; i < 9; i++)\n"
                                    "    {\n"
                                    "        float v = Iy((float(i) - 4.0) / w).r;\n"
                                    "        response.b += kernel28[i+6] * v;\n"
                                    "        response.g += kernel20[i+3] * v;\n"
                                    "        response.r += kernel12[i  ] * v;\n"
                                    "    }\n"
                                    "    for (int i = 12; i < 15; i++)\n"
                                    "    {\n"
                                    "        float v = Iy((float(i) - 7.0) / w).r;\n"
                                    "        response.g += kernel20[i  ] * v;\n"
                                    "        response.b += kernel28[i+3] * v;\n"
                                    "    }\n"
                                    "    for (int i = 18; i < 21; i++)\n"
                                    "    {\n"
                                    "        response.b += kernel28[i] * Iy((float(i) - 10.0) / w).r;\n"
                                    "    }\n"
                                    "    pixel = N(response);\n"
                                    "}\n";

static std::string detHFs = "#ifdef GL_ES\n"
                            "precision highp float;\n"
                            "#endif\n"
                            "out vec3 pixel;\n"
                            "in vec2 texcoords;\n"
                            "uniform sampler2D tgxx;\n"
                            "uniform sampler2D tgyy;\n"
                            "uniform sampler2D tgxy;\n"
                            "\n"
                            "#include normalizer\n"
                            "\n"
                            "void main()\n"
                            "{\n"
                            "    \n"
                            "    vec3 gxx = UN(texture(tgxx, texcoords).rgb);\n"
                            "    vec3 gyy = UN(texture(tgyy, texcoords).rgb);\n"
                            "    vec3 gxy = UN(texture(tgxy, texcoords).rgb);\n"
                            "    vec3 det = gxx * gyy - gxy * gxy;\n"
                            "    pixel = N(det);\n"
                            "}\n";

static std::string nmsxFs = "#ifdef GL_ES\n"
                            "precision highp float;\n"
                            "#endif\n"
                            "out vec3 pixel;\n"
                            "in vec2 texcoords;\n"
                            "uniform sampler2D tex;\n"
                            "uniform float w;\n"
                            "\n"
                            "void main()\n"
                            "{\n"
                            "    vec3 o = texture(tex, texcoords).rgb;\n"
                            "    vec3 px = texture(tex, texcoords + vec2(1.0/w, 0.0f)).rgb;\n"
                            "    vec3 nx = texture(tex, texcoords - vec2(1.0/w, 0.0f)).rgb;\n"
                            "    pixel = o;\n"
                            "    if (o.r <= nx.r || o.r <= px.r)\n"
                            "    {\n"
                            "       pixel.r = 0.0;\n"
                            "    }\n"
                            "    if (o.g <= nx.g || o.g <= px.g)\n"
                            "    {\n"
                            "       pixel.g = 0.0;\n"
                            "    }\n"
                            "    if (o.b <= nx.b || o.b <= px.b)\n"
                            "    {\n"
                            "       pixel.b = 0.0;\n"
                            "    }\n"
                            "}\n";

static std::string nmsyFs = "#ifdef GL_ES\n"
                            "precision highp float;\n"
                            "#endif\n"
                            "out vec3 pixel;\n"
                            "in vec2 texcoords;\n"
                            "uniform sampler2D tex;\n"
                            "uniform float w;\n"
                            "\n"
                            "void main()\n"
                            "{\n"
                            "    \n"
                            "    vec3 o = texture(tex, texcoords).rgb;\n"
                            "    vec3 py = texture(tex, texcoords + vec2(0.0f, 1.0/w)).rgb;\n"
                            "    vec3 ny = texture(tex, texcoords - vec2(0.0f, 1.0/w)).rgb;\n"
                            "    pixel = o;"
                            "    if (o.r <= ny.r || o.r <= py.r)\n"
                            "    {\n"
                            "       pixel.r = 0.0;\n"
                            "    }\n"
                            "    if (o.g <= ny.g || o.g <= py.g)\n"
                            "    {\n"
                            "       pixel.g = 0.0;\n"
                            "    }\n"
                            "    if (o.b <= ny.b || o.b <= py.b)\n"
                            "    {\n"
                            "       pixel.b = 0.0;\n"
                            "    }\n"
                            "}\n";

static std::string nmszFs = "#ifdef GL_ES\n"
                            "precision highp float;\n"
                            "#endif\n"
                            "out vec4 pixel;\n"
                            "in vec2 texcoords;\n"
                            "uniform sampler2D det;\n"
                            "uniform sampler2D tgxx;\n"
                            "uniform sampler2D tgyy;\n"
                            "uniform sampler2D tgxy;\n"
                            "\n"
                            "#include normalizer\n"
                            "\n"
                            "void main()\n"
                            "{\n"
                            "    \n"
                            "    vec3 l = UN(texture(det, texcoords).rgb);\n"
                            "    float gxx = UN(texture(tgxx, texcoords).g);\n"
                            "    float gyy = UN(texture(tgyy, texcoords).g);\n"
                            "    float gxy = UN(texture(tgxy, texcoords).g);\n"
                            "    float tr = gxx + gyy;\n"
                            "    float r = tr * tr / l.g;\n"
                            "    pixel = vec4(vec3(0.0), 1.0);\n"
                            "    if (l.g > 0.0015 && l.g > l.r && l.g > l.b && r < 6.0)\n"
                            "    {\n"
                            "       pixel = vec4(1.0);\n"
                            "    }\n"
                            "}\n";

static std::string extractorFS = "#ifdef GL_ES\n"
                                 "precision highp float;\n"
                                 "precision highp iimage2D;\n"
                                 "#endif\n"
                                 "layout (binding = 0, offset = 0) uniform atomic_uint counter;\n"
                                 "layout (rgba32i) uniform writeonly iimage2D image;\n"
                                 "uniform sampler2D tex;\n"
                                 "uniform float w;\n"
                                 "uniform float h;\n"
                                 "in vec2 texcoords;\n"
                                 "\n"
                                 "void main()\n"
                                 "{\n"
                                 "    ivec2 pos = ivec2(int(w * texcoords.x), int(h * texcoords.y));\n"
                                 "    if (pos.x < 15 || pos.y < 15 || pos.x > int(w)-15 || pos.y > int(h)-15)\n"
                                 "        discard;"
                                 "    float r = texture(tex, texcoords).r;\n"
                                 "    if (r > 0.1)\n"
                                 "    {\n"
                                 "         int i = int(atomicCounterIncrement(counter));\n"
                                 "         if (i < 4096)\n"
                                 "         {\n"
                                 "             imageStore(image, ivec2(i, 0), ivec4(pos, 10, 10));\n"
                                 "         }\n"
                                 "    }\n"
                                 "}\n";


GLuint ImageProcessor::buildShaderFromSource(string source, GLenum shaderType)
{
    // Compile Shader code
    GLuint      shaderHandle = glCreateShader(shaderType);
    string version;
    SLGLState* state = SLGLState::instance();

    if (state->glIsES3())
    {
        version = "#version 320 es\n";
    }
    else
    {
        version = "#version 450\n";
    }

    string completeSrc = version + source;

    Utils::replaceString(completeSrc, "#include texOffsets", textureOfstFct);
    Utils::replaceString(completeSrc, "#include normalizer", normalizedFct);

    const char* src         = completeSrc.c_str();

    glShaderSource(shaderHandle, 1, &src, nullptr);
    glCompileShader(shaderHandle);

    // Check compile success
    GLint compileSuccess;
    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &compileSuccess);

    GLint logSize = 0;
    glGetShaderiv(shaderHandle, GL_INFO_LOG_LENGTH, &logSize);

    GLchar * log = new GLchar[logSize];

    glGetShaderInfoLog(shaderHandle, logSize, nullptr, log);

    if (!compileSuccess)
    {
        Utils::log("Cannot compile shader %s\n", log);
        std::cout << completeSrc << std::endl;
        exit(1);
    }
    return shaderHandle;
}

void ImageProcessor::initShaders()
{
    GLuint vscreenQuad = buildShaderFromSource(screeQuadVs, GL_VERTEX_SHADER);
    GLuint fd2Gdx2     = buildShaderFromSource(hGaussianDx2Fs, GL_FRAGMENT_SHADER);
    GLuint fd2Gdy2     = buildShaderFromSource(vGaussianDy2Fs, GL_FRAGMENT_SHADER);
    GLuint fdGdx       = buildShaderFromSource(hGaussianDxFs, GL_FRAGMENT_SHADER);
    GLuint fdGdy       = buildShaderFromSource(vGaussianDyFs, GL_FRAGMENT_SHADER);
    GLuint fGx         = buildShaderFromSource(hGaussianFs, GL_FRAGMENT_SHADER);
    GLuint fGy         = buildShaderFromSource(vGaussianFs, GL_FRAGMENT_SHADER);
    GLuint fdetH       = buildShaderFromSource(detHFs, GL_FRAGMENT_SHADER);
    GLuint fnmsx       = buildShaderFromSource(nmsxFs, GL_FRAGMENT_SHADER);
    GLuint fnmsy       = buildShaderFromSource(nmsyFs, GL_FRAGMENT_SHADER);
    GLuint fnmsz       = buildShaderFromSource(nmszFs, GL_FRAGMENT_SHADER);
    GLuint fextractor  = buildShaderFromSource(extractorFS, GL_FRAGMENT_SHADER);

    d2Gdx2      = glCreateProgram();
    d2Gdy2      = glCreateProgram();
    dGdx        = glCreateProgram();
    dGdy        = glCreateProgram();
    Gx          = glCreateProgram();
    Gy          = glCreateProgram();
    detH        = glCreateProgram();
    nmsx        = glCreateProgram();
    nmsy        = glCreateProgram();
    nmsz        = glCreateProgram();
    extractor   = glCreateProgram();

    glAttachShader(d2Gdx2, vscreenQuad);
    glAttachShader(d2Gdx2, fd2Gdx2);
    glLinkProgram(d2Gdx2);

    glAttachShader(d2Gdy2, vscreenQuad);
    glAttachShader(d2Gdy2, fd2Gdy2);
    glLinkProgram(d2Gdy2);

    glAttachShader(dGdx, vscreenQuad);
    glAttachShader(dGdx, fdGdx);
    glLinkProgram(dGdx);

    glAttachShader(dGdy, vscreenQuad);
    glAttachShader(dGdy, fdGdy);
    glLinkProgram(dGdy);

    glAttachShader(Gx, vscreenQuad);
    glAttachShader(Gx, fGx);
    glLinkProgram(Gx);

    glAttachShader(Gy, vscreenQuad);
    glAttachShader(Gy, fGy);
    glLinkProgram(Gy);

    glAttachShader(detH, vscreenQuad);
    glAttachShader(detH, fdetH);
    glLinkProgram(detH);

    glAttachShader(nmsx, vscreenQuad);
    glAttachShader(nmsx, fnmsx);
    glLinkProgram(nmsx);

    glAttachShader(nmsy, vscreenQuad);
    glAttachShader(nmsy, fnmsy);
    glLinkProgram(nmsy);

    glAttachShader(nmsz, vscreenQuad);
    glAttachShader(nmsz, fnmsz);
    glLinkProgram(nmsz);

    glAttachShader(extractor, vscreenQuad);
    glAttachShader(extractor, fextractor);
    glLinkProgram(extractor);

    glDeleteShader(vscreenQuad);
    glDeleteShader(fd2Gdx2);
    glDeleteShader(fd2Gdy2);
    glDeleteShader(fdGdx);
    glDeleteShader(fdGdy);
    glDeleteShader(fGx);
    glDeleteShader(fGy);
    glDeleteShader(fdetH);
    glDeleteShader(fnmsx);
    glDeleteShader(fnmsy);
    glDeleteShader(fnmsz);
    glDeleteShader(fextractor);

    d2Gdx2TexLoc = glGetUniformLocation(d2Gdx2, "tex");
    d2Gdx2WLoc   = glGetUniformLocation(d2Gdx2, "w");
    d2Gdy2TexLoc = glGetUniformLocation(d2Gdy2, "tex");
    d2Gdy2WLoc   = glGetUniformLocation(d2Gdy2, "w");
    dGdxTexLoc   = glGetUniformLocation(dGdx, "tex");
    dGdxWLoc     = glGetUniformLocation(dGdx, "w");
    dGdyTexLoc   = glGetUniformLocation(dGdy, "tex");
    dGdyWLoc     = glGetUniformLocation(dGdy, "w");
    GxTexLoc     = glGetUniformLocation(Gx, "tex");
    GxWLoc       = glGetUniformLocation(Gx, "w");
    GyTexLoc     = glGetUniformLocation(Gy, "tex");
    GyWLoc       = glGetUniformLocation(Gy, "w");
    detHGxxLoc   = glGetUniformLocation(detH, "tgxx");
    detHGyyLoc   = glGetUniformLocation(detH, "tgyy");
    detHGxyLoc   = glGetUniformLocation(detH, "tgxy");
    nmsxTexLoc   = glGetUniformLocation(nmsx, "tex");
    nmsxWLoc     = glGetUniformLocation(nmsx, "w");
    nmsyTexLoc   = glGetUniformLocation(nmsy, "tex");
    nmsyWLoc     = glGetUniformLocation(nmsy, "w");
    nmszTexLoc   = glGetUniformLocation(nmsz, "det");
    nmszGxxLoc   = glGetUniformLocation(nmsz, "tgxx");
    nmszGyyLoc   = glGetUniformLocation(nmsz, "tgyy");
    nmszGxyLoc   = glGetUniformLocation(nmsz, "tgxy");

    extractorTexLoc = glGetUniformLocation(extractor, "tex");
    extractorWLoc = glGetUniformLocation(extractor, "w");
    extractorHLoc = glGetUniformLocation(extractor, "h");
    extractorAtomicCounterLoc = 0;//glGetUniformLocation(extractor, "counter");
    extractorKpBufferLoc = glGetUniformLocation(extractor, "image");
}

void ImageProcessor::initVBO()
{
    float vertices[12] = {-1, -1,  0,
                           1, -1,  0,
                           1,  1,  0,
                          -1,  1,  0 };

    GLuint indices[6] = {0, 1, 2, 2, 3, 0};

    //Gen VBO
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), vertices, GL_STATIC_DRAW);

    glGenBuffers(1, &vboi);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(float), indices, GL_STATIC_DRAW);

    // Gen VAO
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void ImageProcessor::setTextureParameters()
{
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);
}

void ImageProcessor::textureRGBAI(int w, int h)
{
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32I, w, h, 0, GL_RGBA_INTEGER, GL_INT, nullptr);
}

void ImageProcessor::textureRGBF(int w, int h)
{
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_HALF_FLOAT, nullptr);
}

void ImageProcessor::textureRF(int w, int h)
{
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, w, h, 0, GL_RED, GL_HALF_FLOAT, nullptr);
}

void ImageProcessor::textureRB(int w, int h)
{
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, w, h, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
}

void ImageProcessor::resetAtomicCounter()
{
    GLuint data = 0;
    glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(GLuint), &data);

    /*
    GLuint * n = (GLuint*)glMapBufferRange(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(GLuint), GL_MAP_WRITE_BIT);
    *n = 0;
    glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);
    */
}

void ImageProcessor::initAtomicCounters()
{
    glGenBuffers(2, atomicCounters);

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomicCounters[0]);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(GLuint), NULL, GL_DYNAMIC_COPY);

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomicCounters[1]);
    glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(GLuint), NULL, GL_DYNAMIC_COPY);
}

void ImageProcessor::initTextureBuffers(int width, int height)
{
    glGenTextures(12, renderTextures);

    glBindTexture(GL_TEXTURE_2D, renderTextures[0]);
    setTextureParameters();
    textureRB(width, height);

    int i = 1;

    for (; i < 10; i++)
    {
        glBindTexture(GL_TEXTURE_2D, renderTextures[i]);
        setTextureParameters();
        textureRGBF(width, height);
    }

    for (; i < 12; i++)
    {
        glBindTexture(GL_TEXTURE_2D, renderTextures[i]);
        setTextureParameters();
        textureRF(width, height);
    }
}

void ImageProcessor::initKeypointBuffers(int nb_elements)
{
    glGenTextures(1, &outTexture);
    glBindTexture(GL_TEXTURE_2D, outTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    textureRGBAI(nb_elements, 1);

    glGenBuffers(2, pbo);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo[0]);
    glBufferData(GL_PIXEL_PACK_BUFFER, nb_elements * 4 * sizeof(int), 0, GL_DYNAMIC_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo[1]);
    glBufferData(GL_PIXEL_PACK_BUFFER, nb_elements * 4 * sizeof(int), 0, GL_DYNAMIC_READ);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
}

void ImageProcessor::initFBO()
{
    glGenFramebuffers(12, renderFBO);

    for (int i = 0; i < 12; i++)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderTextures[i], 0);
    }

    glGenFramebuffers(1, &outFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, outFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, outTexture, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ImageProcessor::gxx(int w, int h)
{
    glUseProgram(d2Gdx2);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[D2GDX2]);

    glUniform1i(d2Gdx2TexLoc, INPUT);
    glUniform1f(d2Gdx2WLoc, (float)w);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

    glUseProgram(Gy);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[GXX]);

    glUniform1i(GyTexLoc, D2GDX2);
    glUniform1f(GyWLoc, (float)h);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
}

void ImageProcessor::gyy(int w, int h)
{
    glUseProgram(d2Gdy2);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[D2GDY2]);

    glUniform1i(d2Gdy2TexLoc, INPUT);
    glUniform1f(d2Gdy2WLoc, (float)h);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

    glUseProgram(Gx);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[GYY]);

    glUniform1i(GxTexLoc, D2GDY2);
    glUniform1f(GxWLoc, (float)w);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
}

void ImageProcessor::gxy(int w, int h)
{
    glUseProgram(dGdx);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[DGDX]);

    glUniform1i(dGdxTexLoc, INPUT);
    glUniform1f(dGdxWLoc, (float)w);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

    glUseProgram(dGdy);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[GXY]);

    glUniform1i(GyTexLoc, DGDX);
    glUniform1f(GyWLoc, (float)h);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
}

void ImageProcessor::det(int w, int h)
{
    glUseProgram(detH);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[DETH]);

    glUniform1i(detHGxxLoc, GXX);
    glUniform1i(detHGyyLoc, GYY);
    glUniform1i(detHGxyLoc, GXY);

    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
}

void ImageProcessor::nms(int w, int h)
{
    glUseProgram(nmsx);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[NMSX]);
    glUniform1i(nmsxTexLoc, DETH);
    glUniform1f(nmsxWLoc, (float)w);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

    glUseProgram(nmsy);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[NMSY]);
    glUniform1i(nmsyTexLoc, NMSX);
    glUniform1f(nmsyWLoc, (float)h);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

    glUseProgram(nmsz);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[NMSZ]);
    glUniform1i(nmszTexLoc, NMSY);
    glUniform1i(nmszGxxLoc, GXX);
    glUniform1i(nmszGyyLoc, GYY);
    glUniform1i(nmszGxyLoc, GXY);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
}

void ImageProcessor::extract(int w, int h, int curr)
{
    glUseProgram(extractor);
    glBindFramebuffer(GL_FRAMEBUFFER, renderFBO[EXTRACTOR]);
    glUniform1f(extractorWLoc, (float)w);
    glUniform1f(extractorHLoc, (float)h);
    glUniform1i(extractorTexLoc, NMSZ);

    glBindImageTexture(0, outTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32I);
    glUniform1i(extractorKpBufferLoc, 0);

    glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, extractorAtomicCounterLoc, atomicCounters[curr]);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboi);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);

    glMemoryBarrier(GL_FRAMEBUFFER_BARRIER_BIT);
}

void ImageProcessor::init(int w, int h)
{
    m_w = w;
    m_h = h;
    curr = 1;
    ready = 0;
    initShaders();
    initAtomicCounters();
    initVBO();
    initTextureBuffers(w, h);
    initKeypointBuffers(4096);
    initFBO();
    GET_GL_ERROR;
}

ImageProcessor::ImageProcessor(){}

ImageProcessor::ImageProcessor(int w, int h)
{
    init(w, h);
}

ImageProcessor::~ImageProcessor()
{
    glDeleteTextures(12, renderTextures);
    glDeleteFramebuffers(12, renderFBO);
    glDeleteTextures(1, &outTexture);
    glDeleteBuffers(2, atomicCounters);
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(2, pbo);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &vboi);

    glDeleteProgram(d2Gdx2);
    glDeleteProgram(d2Gdy2);
    glDeleteProgram(dGdx);
    glDeleteProgram(dGdy);
    glDeleteProgram(Gx);
    glDeleteProgram(Gy);
    glDeleteProgram(detH);
    glDeleteProgram(nmsx);
    glDeleteProgram(nmsy);
    glDeleteProgram(nmsz);
    glDeleteProgram(extractor);
}

void ImageProcessor::gpu_kp()
{
    glDisable(GL_DEPTH_TEST);

    ready = curr;
    curr = (curr+1) % 2; //Set rendering buffers
    HighResTimer t;
    t.start();

    AVERAGE_TIMING_START("DETECT KEYPOINTS");

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomicCounters[curr]);
    resetAtomicCounter();

    glBindFramebuffer(GL_FRAMEBUFFER, outFBO);
    glClear(GL_COLOR_BUFFER_BIT);

    SLVec4i wp = SLGLState::instance()->getViewport();
    SLGLState::instance()->viewport(0, 0, m_w, m_h);

    for (int i = 0; i < 12; i++)
    {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, renderTextures[i]);
    }

    //blur(m_w, m_h);
    gxx(m_w, m_h);
    gyy(m_w, m_h);
    gxy(m_w, m_h);
    det(m_w, m_h);
    nms(m_w, m_h);
    extract(m_w, m_h, curr);

    glUseProgram(0);
    glEnable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    SLGLState::instance()->viewport(wp.x, wp.y, wp.z, wp.w);

    AVERAGE_TIMING_STOP("DETECT KEYPOINTS");
    GET_GL_ERROR;
}

void ImageProcessor::readResult(std::vector<cv::KeyPoint> &kps)
{
    HighResTimer t;
    t.start();
    AVERAGE_TIMING_START("PBO");

    /* Start asynchronous copy pixel to curr pbo */

    glBindFramebuffer(GL_FRAMEBUFFER, outFBO);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo[curr]);
    glReadPixels(0, 0, 4096, 1, GL_RGBA_INTEGER, GL_INT, (GLvoid*)0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    /*
    glBindTexture(GL_TEXTURE_2D, outTexture);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo[curr]);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA_INTEGER, GL_INT, (GLvoid*)0);
    */

    /* Continue processing without stall */

    glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, atomicCounters[ready]);
    int * n = (int*)glMapBufferRange(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(int), GL_MAP_READ_BIT);
    Utils::log("atomic counter value %d\n", *n);
    glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

    /* Read pixels from ready pbo */
    glBindBuffer(GL_PIXEL_PACK_BUFFER, pbo[ready]); //Read pixel from ready pbo
    int * data = (int*)glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, 4096 * 4 * sizeof(int), GL_MAP_READ_BIT);

    if (data)
    {
        int i = 0;
        for (; i < 4096; i++)
        {
            int idx = i * 4;
            if (data[idx] == 0) { break; }
            kps.push_back(cv::KeyPoint(cv::Point2f(data[idx], data[idx+1]), 1));
        }
            //Utils::log("AAAAAAAA  %d\n", i);
    }
    else
        std::cout << "null data" << std::endl;

    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

    AVERAGE_TIMING_STOP("PBO");
    GET_GL_ERROR;
}

