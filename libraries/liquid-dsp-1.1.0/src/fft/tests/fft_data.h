/*
 * Copyright (c) 2007, 2008, 2009, 2010 Joseph Gaeddert
 * Copyright (c) 2007, 2008, 2009, 2010 Virginia Polytechnic
 *                                      Institute & State University
 *
 * This file is part of liquid.
 *
 * liquid is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * liquid is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with liquid.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// autotest fft data
//

#ifndef __LIQUID_AUTOTEST_FFT_DATA_H__
#define __LIQUID_AUTOTEST_FFT_DATA_H__

#include <complex.h>
#define J _Complex_I

// 4-point fft data
float complex x4[] = {
   -0.995988 +     0.363214*J,
   -0.447631 +    -0.094074*J,
    0.494805 +     0.293037*J,
    0.865135 +    -0.436518*J
};

float complex test4[] = {
   -0.083680 +     0.125659*J,
   -1.148348 +     1.382943*J,
   -0.918686 +     1.186843*J,
   -1.833237 +    -1.242589*J
};


// 5-point fft data
float complex x5[] = {
   -0.406017 +     1.455046*J,
    0.069610 +     1.476957*J,
   -1.476361 +    -0.533247*J,
    1.449419 +     1.037569*J,
    0.090513 +    -1.596866*J
};

float complex test5[] = {
   -0.272837 +     1.839459*J,
    1.665337 +     2.749597*J,
    2.756797 +    -1.062396*J,
   -3.844568 +     4.478194*J,
   -2.334816 +    -0.729623*J
};

// 6-point fft data
float complex x6[] = {
    0.420909 +     2.868720*J,
    0.118262 +    -1.028969*J,
    0.601747 +     1.365914*J,
    0.671793 +    -0.745299*J,
   -1.372470 +    -1.271472*J,
   -1.541897 +    -0.506349*J
};

float complex test6[] = {
   -1.101656 +     0.682544*J,
    1.254100 +    -0.348323*J,
   -0.546765 +     3.115840*J,
    0.402028 +     5.243778*J,
    4.926526 +     2.571877*J,
   -2.408781 +     5.946601*J
};

// 7-point fft data
float complex x7[] = {
   -1.088121 +    -1.161896*J,
   -1.699878 +     0.787537*J,
   -0.933514 +    -0.731976*J,
    0.532032 +    -0.046917*J,
   -1.210747 +    -0.230806*J,
    1.637651 +    -1.490334*J,
   -0.224449 +    -2.392955*J
};

float complex test7[] = {
   -2.987026 +    -5.267346*J,
    1.472635 +     1.485945*J,
    0.910446 +     2.709827*J,
    2.202021 +    -4.108388*J,
    0.269354 +     2.029885*J,
   -4.345436 +    -0.660991*J,
   -5.138844 +    -4.322200*J
};

// 8-point fft data
float complex x8[] = {
   -0.691190 +     0.619688*J,
    0.440635 +     0.246065*J,
    0.974030 +     1.251579*J,
   -0.382260 +     0.142588*J,
   -0.458303 +     1.967058*J,
   -0.834497 +    -1.028022*J,
   -0.573837 +    -1.316674*J,
    0.072922 +    -0.295330*J
};

float complex test8[] = {
   -1.452501 +     1.586952*J,
    4.769453 +    -2.883768*J,
   -2.178901 +     2.736365*J,
   -2.814086 +    -0.970556*J,
   -0.046100 +     3.456350*J,
   -0.098722 +    -2.906705*J,
   -0.920471 +     2.567316*J,
   -2.788193 +     1.371550*J
};

// 9-point fft data
float complex x9[] = {
   -1.620235 +    -0.517400*J,
    1.457078 +     0.141899*J,
    0.181402 +     1.138319*J,
   -0.472447 +     0.709261*J,
   -0.518349 +     0.641070*J,
    1.022213 +    -0.256426*J,
    0.572218 +    -1.505036*J,
    0.273838 +     0.231156*J,
   -0.862207 +     1.453261*J
};

float complex test9[] = {
    0.033510 +     2.036104*J,
    0.666204 +     1.010644*J,
   -5.084334 +    -4.982141*J,
   -3.441496 +    -3.742260*J,
   -1.790737 +    -1.967710*J,
   -1.795039 +     0.962509*J,
   -1.153406 +    -2.233368*J,
    1.867082 +     3.312648*J,
   -3.883901 +     0.946976*J
};

// 10-point fft data
float complex x10[] = {
   -1.658601 +     0.894400*J,
   -0.412701 +    -2.502463*J,
   -0.148492 +    -0.029669*J,
    1.237217 +     0.480954*J,
   -1.010974 +    -0.605520*J,
    0.811898 +    -0.280680*J,
   -0.674672 +     1.452293*J,
   -0.781364 +    -0.358109*J,
    0.249896 +     0.757794*J,
    0.465506 +    -0.154439*J
};

float complex test10[] = {
   -1.922287 +    -0.345440*J,
   -3.714205 +    -2.299436*J,
   -3.034164 +     1.301950*J,
   -6.941486 +     3.875485*J,
    2.022830 +     0.360958*J,
   -4.563398 +     5.284034*J,
   -0.729842 +     4.321259*J,
    1.499574 +    -0.339298*J,
   -0.570055 +    -2.570131*J,
    1.367018 +    -0.645386*J
};

// 16-point fft data
float complex x16[] = {
    0.655013 +     0.461657*J,
   -0.176225 +    -0.994668*J,
    1.069081 +    -1.974775*J,
    0.030636 +    -1.411363*J,
   -0.133946 +    -0.392896*J,
    1.487670 +    -1.532182*J,
   -0.011962 +    -2.709565*J,
    0.680375 +     1.824683*J,
   -0.292712 +     0.771407*J,
    0.022302 +     1.950677*J,
    0.008483 +     0.602351*J,
    1.049228 +     0.384728*J,
    1.167451 +     1.025550*J,
   -0.408801 +    -0.534208*J,
    0.235000 +    -0.452407*J,
    0.427950 +    -0.081462*J
};

float complex test16[] = {
    5.809543 +    -3.062474*J,
   -7.475728 +    -5.429231*J,
    0.445329 +     4.733469*J,
    0.652973 +    -2.793290*J,
   -1.731764 +     7.663358*J,
   -0.388396 +     3.198601*J,
   -1.430585 +    -1.748964*J,
    5.262217 +     2.693869*J,
   -0.416726 +    -2.274882*J,
    1.546745 +     5.809368*J,
    1.791361 +    -5.241699*J,
   -4.606511 +    -1.127157*J,
    1.922171 +     5.136870*J,
    4.434496 +     0.387851*J,
   -3.490916 +     4.658839*J,
    8.156005 +    -5.218009*J
};

// 20-point fft data
float complex x20[] = {
   -2.257949 +     0.143633*J,
    0.677479 +    -0.091418*J,
    0.008673 +    -1.342362*J,
   -1.062541 +     0.181155*J,
   -0.542167 +    -0.849229*J,
   -0.411084 +     1.148501*J,
    1.630756 +    -0.477277*J,
    0.615519 +     1.687153*J,
   -1.623097 +     1.350753*J,
   -0.464235 +    -0.560898*J,
   -1.795185 +    -0.901238*J,
    0.383254 +     0.671371*J,
    0.182629 +     0.568865*J,
    1.055019 +    -0.703973*J,
   -1.174599 +     1.070719*J,
   -0.516963 +    -0.626138*J,
    0.786249 +     0.465326*J,
   -0.790637 +    -0.772840*J,
   -0.535621 +     0.426022*J,
    1.815367 +     0.628271*J
};

float complex test20[] = {
   -4.019132 +     2.016397*J,
    1.162837 +    -1.651680*J,
   -5.645625 +    -0.530671*J,
    2.714870 +     8.460059*J,
   -2.668866 +    -6.036911*J,
   -5.110798 +     4.071579*J,
    0.843151 +     3.777260*J,
   -8.502056 +     0.827617*J,
   -6.940596 +    -0.680922*J,
    7.014094 +     0.715064*J,
   -6.621490 +    -1.105973*J,
   -7.345917 +    -2.777689*J,
   -9.386064 +    -0.313983*J,
   -2.373136 +     1.099409*J,
   -5.263340 +    -7.597648*J,
    1.934078 +     1.735391*J,
   -1.891243 +     3.839209*J,
    5.866380 +     0.460586*J,
    1.061865 +    -0.942806*J,
    0.012005 +    -2.491629*J
};

// 32-point fft data
float complex x32[] = {
  0.961067864909 +   0.286335976496*J,
  0.286620373507 +  -1.102677365222*J,
  2.325318943438 +   1.027915910835*J,
 -1.315325966542 +   0.848609514062*J,
 -0.237551072471 +  -0.495697802556*J,
 -1.641791207710 +   0.353159752894*J,
 -0.157082118405 +  -1.650538558328*J,
  0.753530187921 +   0.177783281419*J,
 -1.220432057527 +   0.668419091887*J,
  0.149532110219 +  -0.147897988156*J,
  1.689623713783 +   0.680482332029*J,
  1.012148016504 +  -2.125406594771*J,
 -0.269259848704 +   0.747010004865*J,
  1.125730017419 +   0.268147563672*J,
 -1.571735839429 +  -0.408160795892*J,
  0.640934293631 +   0.567325657726*J,
 -0.160379407251 +  -1.275369007587*J,
  0.634406478832 +   0.854559707293*J,
  1.381740692078 +  -0.631444125756*J,
  0.476916028207 +   0.040060743280*J,
  0.361888381963 +  -0.243726531633*J,
  0.549327875093 +   0.740571341856*J,
 -1.801192169035 +  -2.490384124468*J,
  0.016540491640 +  -0.126387636035*J,
  0.344208443719 +  -0.298722560031*J,
  0.599658245602 +   1.408534651569*J,
  0.697314642427 +  -0.264810930006*J,
  2.201161771758 +   0.905756543393*J,
 -0.304406393087 +  -0.135493365162*J,
 -0.121110444045 +   1.664763081568*J,
  1.579435504748 +   0.808381920570*J,
  0.854073659033 +  -0.452695831243*J
};

float complex test32[] = {
  9.840911212224 +   0.198403858569*J,
  0.372618970380 +   7.435693662330*J,
  0.076404578009 +   7.282339389029*J,
  1.820748529445 +   0.656869874296*J,
  4.316484387888 + -10.929557628048*J,
  6.313222356978 +   0.724181949523*J,
 -2.309394664801 +  -8.519894418185*J,
 -5.374616889095 +  -4.813787879368*J,
 -0.464172390412 +   5.238919210533*J,
 -5.515309916239 +   5.160595487673*J,
 -2.860035925225 +  -4.276652329194*J,
 -1.576150125343 +  -4.195869855930*J,
 -7.120168110622 +   7.305115699457*J,
  1.802861502166 +   9.955901782278*J,
  9.066578438666 +   8.180027098275*J,
 -0.122576870238 +   5.199728432163*J,
 -2.603792649909 +  -7.550008988042*J,
 15.501227509948 +   0.062059053352*J,
 -1.226521458316 +  -4.087437204857*J,
  1.658557984261 +  10.232569797312*J,
  5.536792654850 +  -6.142445209145*J,
 -4.607992311225 +  10.035833135288*J,
 -2.731439208807 +   1.716821202405*J,
 -0.533472442550 +  -6.009621202383*J,
 -8.872402525697 +  -0.876290855939*J,
 -2.015647027690 +  -1.765683735748*J,
  5.314037195827 +  -7.145182311903*J,
  2.292360436768 +  -1.900471664442*J,
 -1.237933827518 +   7.801171918736*J,
  4.857730308302 +  -6.597817452060*J,
  8.085667616379 +  -4.019857929150*J,
  3.069594338684 +   0.807098361039*J
};

float complex x64[] = {
  0.018433802161 +   1.312209206080*J,
  2.158993707240 +  -1.426595952083*J,
 -1.058246453930 +  -0.083532510021*J,
  0.284642267824 +  -0.353585231353*J,
  1.024288545329 +   0.670848790173*J,
  1.099747635395 +   0.408924622105*J,
 -0.813701638498 +  -0.333269715021*J,
 -0.623017439662 +   1.323390342880*J,
 -0.338351606501 +  -1.134432243276*J,
  1.080139708673 +   0.794506792049*J,
  0.465380255690 +   0.667525184347*J,
  0.299331623102 +  -0.957608688378*J,
 -0.694955933076 +  -0.208720098199*J,
  0.370691404618 +   0.833424242145*J,
 -0.159596162745 +  -1.117277876037*J,
 -0.354344124916 +  -0.579183100042*J,
  0.479588760178 +  -0.674895239604*J,
  0.865492259190 +   1.665726619624*J,
 -1.288035328585 +  -0.392681907390*J,
  0.038451073083 +  -1.613792877580*J,
  2.257998205535 +  -0.279479408966*J,
  1.048656703977 +   0.701593281206*J,
  0.493487990178 +  -1.040176731610*J,
 -2.171150746859 +   2.092795197571*J,
  0.268582995273 +  -1.212979630073*J,
 -0.225057526413 +   0.145851026202*J,
  0.476715762402 +   0.468876974239*J,
 -1.067369855919 +  -2.158750381452*J,
 -0.591970305742 +   0.230725856824*J,
  1.133754113738 +   0.571421937539*J,
  0.476292162630 +   0.088594763290*J,
 -0.510461712166 +   0.486897877290*J,
 -0.069410497872 +   0.087237763221*J,
 -0.816043734211 +  -0.942055624944*J,
  1.488697871336 +  -0.680147165475*J,
  0.319109239734 +   1.415714154272*J,
  0.027243479106 +   2.144527588870*J,
 -1.217757631026 +  -0.884074754702*J,
  0.543931309899 +   0.690259574502*J,
  0.083826747171 +   0.688592522379*J,
  0.624811653986 +  -0.507151646195*J,
  0.697612396827 +  -1.190289368728*J,
  0.198205593558 +   0.401473821155*J,
  0.542762562817 +  -0.014683757285*J,
  3.960384191601 +   1.229211978532*J,
  0.161268675259 +   1.772438208096*J,
  2.214322448540 +   0.769350666829*J,
 -1.974419571901 +   1.041367239853*J,
  0.995082900655 +  -0.669239939997*J,
  0.552406517899 +   0.508260831526*J,
 -0.456658258747 +  -2.287526572942*J,
  1.050958221560 +  -0.387708314673*J,
 -1.433201704907 +   0.178309571342*J,
 -1.264609001540 +  -1.675585525157*J,
  1.700631442997 +   0.906348048279*J,
  0.688230071270 +  -1.488458205243*J,
  1.944176953078 +  -1.646485784054*J,
 -0.085953383725 +  -0.466975671912*J,
 -1.596742557356 +  -0.689411588310*J,
  0.204330017630 +  -0.006911022228*J,
 -0.064305007301 +  -0.442729791071*J,
  0.258356597916 +   1.165180662325*J,
  1.059111991139 +   0.345921860019*J,
 -0.162312968292 +   1.589918282697*J
};

float complex test64[] = {
 14.618456708303 +  -0.148970836538*J,
  0.173743766245 +   3.116445605276*J,
  6.440387759045 +   0.180616164784*J,
 10.008422766207 +   8.701763871919*J,
 -2.816340431319 +   7.043152817271*J,
-16.536149412403 +  -7.339456661390*J,
 10.839608299671 +  -2.839557585815*J,
  0.174942261417 +   5.049926468150*J,
-15.556384475630 +  -3.444941405882*J,
  9.431648943102 +   2.532813321428*J,
-10.846203725568 +   0.995525347083*J,
-13.446031379727 +  14.244736202656*J,
 -8.111592876806 +  -0.780629551228*J,
 15.598080928809 +  -7.531970782855*J,
-14.857449430787 +  -1.302127572458*J,
 12.522236701487 +   5.975833542693*J,
  5.568357289580 +  -7.806502891586*J,
 -3.924945245609 +  -9.605778506919*J,
-10.485885842253 +   5.132265750335*J,
  0.348109092406 +  -4.703648753883*J,
  8.675716641276 +  11.539549043034*J,
-14.078544405908 +  -6.889406076012*J,
 -6.638176673281 +   2.139383332381*J,
 -3.007119092566 + -11.476081114143*J,
 -4.006614852801 + -25.691267576574*J,
  5.264305961985 +   7.877965529477*J,
 -0.233119828709 +  18.389191519322*J,
-20.452542658687 +   0.164273418043*J,
 -4.372213889400 +  -4.928918115119*J,
  7.159608279657 +  -0.443728982721*J,
 -7.512154999583 +   5.287801441592*J,
 -5.923851714245 + -10.902498040717*J,
  9.685929011718 +  -6.268461564542*J,
-17.980492003469 +   6.606944935186*J,
 -2.447446806748 +   9.244449470687*J,
  2.325181031944 +  -4.245371626971*J,
 -0.257777557356 +   0.840658066218*J,
  5.830279872010 +  13.262464329851*J,
  4.615003645501 +  -5.732114060914*J,
  7.843750973451 +  18.254814333292*J,
  8.620902747157 +   2.078402722617*J,
-10.800186671850 +  -0.256073316712*J,
  1.205732676510 +   2.788485774582*J,
 -2.845912797193 +   0.683104456189*J,
 -4.870712354761 +  18.260032402943*J,
  3.527319462534 +   5.531290471011*J,
  2.626273014115 +   6.004069061572*J,
 -4.483365988782 +  -6.682177754091*J,
  3.760842716415 +  10.531763187093*J,
  8.155095238193 +   4.716814598080*J,
  6.546192733933 +  -2.848147012748*J,
 18.252470915818 +  10.674058306788*J,
 -2.280824394118 +   1.733382128356*J,
 -9.587922073841 +  -0.080802582028*J,
  2.395656077558 +  -6.795354192934*J,
 -3.163305087462 +  -7.175026418322*J,
  8.691830542923 +  -4.815921745777*J,
 13.272304850513 +   7.860941443988*J,
  7.358208378680 +   3.476669384107*J,
  1.432029426855 +   1.084683568545*J,
  5.429544616774 +   2.743661954926*J,
  5.810876516849 +   8.488986007710*J,
-13.416998982797 +   9.776157560858*J,
  1.910979143300 +  -8.296753622023*J
};


#endif // __LIQUID_AUTOTEST_FFT_DATA_H__