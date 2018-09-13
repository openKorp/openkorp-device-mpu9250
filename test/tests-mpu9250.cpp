/*
 * Copyright (C) 2018  Björnborg Nguyen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <eigen3/Eigen/Dense>
#include "calibration.hpp"
// #include "MPU9250Device.hpp"

// #include <string>
// #include <vector>
// #include <cmath>

TEST_CASE("Test of ellipse fitting") {
  Eigen::MatrixXf data(3,97);
  data << 
     1.61198118760168f,    0.116166279667157f,     1.37745970240439f,     2.12579185001451f,  -0.0139913834666441f,    0.479454396714228f,   -0.133176263855539f,   -0.248007189101679f,     1.16070045185352f,     1.92124610050549f,
     0.754659267600348f,     2.28054967462401f,     1.00665168670545f,    0.263537891754266f,  -0.0726192237823238f,  -0.0324382894827453f,     2.07745304269637f,     0.41035548841023f,  -0.0714360866017012f,     2.08102701936673f,
     0.651123369580104f,     2.10311192197796f,   -0.116851258596116f,     1.41004229933222f,   0.0554435586147841f,     1.41415483498668f,     0.17287889460788f,     2.25193786755802f,    0.810791156473705f,     1.41647835766567f,
     1.6239720953252f,    0.450809945338356f,     1.23561696645382f,     1.84819164195577f,     1.66946125690736f,     0.57412807941846f,     2.05179575133004f,    0.743135237703507f,    0.561882018169486f,    0.345267779311256f,
     1.20132070572984f,     1.03753680453062f,     1.88115208977834f,     1.27931356606285f,     2.23058239333285f,     1.58435914276844f,     1.43859828595147f,     1.25485742299397f,     0.81552129030799f,    0.634870304664533f,
     0.49727373075039f,     1.06769833916965f,     2.17297098060148f,   -0.126217313494706f,     1.04197909477869f,     1.40785324919569f,    0.272382996334462f,     1.31491228512091f,     1.13551794757232f,     1.47950612704712f,
     1.4766472883219f,    0.770559290758257f,   0.0669293432880478f,     1.72346691960786f,    0.654250136386061f,    0.514232701978415f,   -0.203604869382458f,     1.56094612513406f,     1.42168603192627f,     1.16833237908675f,
     0.938255999136429f,    0.664717240888669f,     0.62792029441851f,    0.257622152875861f,    0.830608192049032f,    0.165191698638176f,     1.93563030840817f,   0.0723906751755381f,   -0.178522111897695f,    0.837831051876572f,
     1.42921051151189f,   0.0906861029764334f,     2.24352990149242f,     2.06070581362066f,    0.515867904358476f,     1.56523942194351f,    0.757345568093405f,    0.701395649841629f,     1.35526282092312f,    0.706157334297817f,
     1.17120521307765f,    0.426604118262741f,     1.12860835991272f,     1.35091971336691f,    0.868182189163833f,    0.246128229039514f,     2.03429054292813f,
     0.526456265248584f,   -0.295166016004031f,  -0.0331839324710979f,   -0.140207139478516f,   -0.117100314755407f,   -0.128796978672494f,  -0.0107330492509607f,    0.126890522824529f,   -0.318588067092474f,    0.351439513970046f,
    -0.520597415498746f,    0.119283586263757f,   -0.546754430991756f,   -0.405773729525267f,   0.0765267357523429f,   -0.303883469918215f,    0.109274591807395f,    0.399118190796267f,  -0.0975050187501569f,   -0.075668410227923f,
     0.131639334482692f,   -0.120935847000652f,   -0.087839408898887f,   -0.113455717736977f,   -0.027342199044563f,   -0.505738047464702f,    0.445911027514997f,   0.0636081455865014f,   -0.442663195552542f,   0.0106052223029147f,
    -0.365055039245074f,    0.423634776611014f,    0.216536388734513f,   -0.121912185902258f,  -0.0439400425051684f,   -0.470591180409091f,   0.0964219809811331f,   -0.185987439446818f,   -0.514178598605926f,   -0.456321143401121f,
    -0.547638975220095f,   -0.525964161973604f,   -0.386514457158114f,   0.0326996059373084f,   0.0659979485635329f,   -0.224007326139729f,    0.522740306395423f,     0.28973128262249f,   -0.571830382208294f,    -0.51108855186783f,
    -0.288339967619623f,   0.0936782369111073f,    0.158077103915553f,    -0.18317560285704f,   -0.232563861862883f,   -0.548904648608052f,    0.151952242715762f,    0.526235970830567f,   -0.557404013035528f,    0.236797889374432f,
     0.387895706302086f,    0.510407721032354f,   0.0853652737472146f,    0.376673509305259f,    0.498458174973809f,   -0.521978541378096f,   -0.132113124791827f,   0.0941326678819152f,    0.544824885281345f,    0.116332853122103f,
    -0.545047672303658f,   0.0423267287576623f,   -0.526573794814914f,    0.173907178787963f,   -0.121232585744403f,   -0.376621811903874f,    0.372949340071221f,    -0.18334305727473f,  -0.0247914973681826f,     0.55372378689361f,
    -0.348831018284685f,   -0.335269124034979f,   0.0525012006822294f,   -0.166913540438375f,    0.181347538532441f,   -0.480640377052476f,  -0.0731092474863395f,    0.510366329559601f,     0.51070388369657f,    0.533091062573481f,
     0.457420623238537f,   -0.387133225771541f,    0.211488150631022f,   0.0840335464935551f,    0.395306983325694f,  -0.0751367806931684f,    0.259134455864402f,
    -0.0167667333232755f,   0.0502157071287836f,   -0.127189182372086f,     0.02762785037502f,  -0.0780427512375246f,   -0.119913974938335f,   -0.054253424608115f,  -0.0230082533968829f,   -0.106743140856977f,   0.0485093384749353f,
    -0.001948231252352f,   0.0138082765000345f,   0.0473183999792948f,  -0.0656022733234014f,  -0.0641822647459884f,   0.0430337444264692f,  -0.0734000922684688f,   0.0751097319702252f,  -0.0771264881063276f,   0.0793238499779205f,
     0.140290426365467f,  -0.0498581429606944f,    -0.04188815016881f,     0.13041914304663f,  -0.0894459021420353f, -0.00161397824539655f,   0.0355774614691517f,   0.0371418708537101f,  -0.0871529236836794f,   -0.132908581248576f,
    -0.0773465940183895f,  -0.0739055131189676f,   -0.137379714469024f,    0.093481692516594f,    0.108678806429821f,   0.0538097713212018f,   0.0856802152119496f,   -0.123273363045736f,  -0.0246728508455965f,   0.0072883685089753f,
    -0.0217949326923419f,   -0.032010385570592f,  -0.0122521080005908f,   -0.141148453478616f,  -0.0378093283357055f,    0.100801475522943f,  -0.0143782400210615f,   -0.119664141981061f,   0.0334495450981578f,   0.0212981541603856f,
    -0.105623613731857f,    0.137424838797565f,   0.0253890827585579f,   0.0573312789425389f,    0.127626929548716f,  -0.0285099147642516f,    0.097278342475926f,   0.0333379955961899f,  -0.0217560491320672f,   -0.113693368863024f,
     0.0924109921631313f,   0.0586393269895709f,   0.0819084057302124f,   0.0727929320926677f,   0.0637129749047576f,  -0.0100413808940223f,  -0.0180175005765486f,    0.127042910012327f,   -0.020370957718689f,    0.133425298750842f,
     0.0572390168732735f,   -0.127170333287484f,   0.0455329943961313f,  -0.0937895726939333f,    0.141336205662936f,   0.0687642254838785f,   0.0449502541366567f,   0.0789616926270105f,  -0.0519479845498605f,   0.0337422782978063f,
    -0.0972974356338936f,   -0.063489582694876f,  -0.0544049124235624f,  -0.0642089919739322f,   -0.128401455511554f,   -0.011543448068374f,    0.145394035421433f,  -0.0214464647186219f,   0.0646888378033781f,  -0.0472069399674195f,
    -0.0714662763678762f,   0.0875757759445221f,    0.138731149003887f,     0.13021114916364f,    -0.10464185617476f,   -0.121044528329135f,   -0.030457508342093f;
  
  Calibration calibration;
  calibration.ellipsoidFit(data);

  Eigen::Vector3f centerRef;
  centerRef << 
    0.998581f,
    0.00109018f,
   -0.000542462f;
  Eigen::Vector3f radiusRef;
  radiusRef <<
    0.149424f,
    0.594446f,
    1.33055f;
  Eigen::Matrix3f rotationRef;
  rotationRef <<
    0.00209929f, -0.00335381f,    0.999992f,
   -0.00228998f,    0.999992f,  0.00335862f,
    0.999995f,  0.00229701f, -0.00209159f;
  rotationRef = rotationRef.transpose().eval();
  // std::cout << (centerRef - calibration.getCenter()).array().abs().sum() << std::endl;
  REQUIRE((centerRef - calibration.getCenter()).array().abs().sum() < 0.0001f);
  REQUIRE((radiusRef - calibration.getRadius()).array().abs().sum() < 0.0001f);
  REQUIRE((rotationRef - calibration.getRotation()).array().abs().sum() < 0.0001f);
  
}
