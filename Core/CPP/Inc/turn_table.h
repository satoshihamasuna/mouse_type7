/*
 * turn_table.h
 *
 *  Created on: 2023/06/21
 *      Author: sato1
 */

#ifndef CPP_INC_TURN_TABLE_H_
#define CPP_INC_TURN_TABLE_H_

const static float accel_table[1001]=
{
			0.000000,
			0.000043,
			0.000171,
			0.000383,
			0.000680,
			0.001060,
			0.001523,
			0.002068,
			0.002695,
			0.003404,
			0.004193,
			0.005061,
			0.006009,
			0.007036,
			0.008140,
			0.009322,
			0.010581,
			0.011915,
			0.013324,
			0.014808,
			0.016366,
			0.017997,
			0.019700,
			0.021475,
			0.023320,
			0.025236,
			0.027221,
			0.029274,
			0.031396,
			0.033584,
			0.035838,
			0.038158,
			0.040542,
			0.042991,
			0.045502,
			0.048075,
			0.050710,
			0.053406,
			0.056161,
			0.058975,
			0.061847,
			0.064776,
			0.067762,
			0.070803,
			0.073899,
			0.077049,
			0.080252,
			0.083507,
			0.086813,
			0.090170,
			0.093577,
			0.097032,
			0.100535,
			0.104086,
			0.107682,
			0.111324,
			0.115011,
			0.118741,
			0.122514,
			0.126328,
			0.130184,
			0.134081,
			0.138016,
			0.141991,
			0.146003,
			0.150051,
			0.154136,
			0.158256,
			0.162411,
			0.166599,
			0.170819,
			0.175072,
			0.179356,
			0.183669,
			0.188012,
			0.192384,
			0.196784,
			0.201210,
			0.205663,
			0.210141,
			0.214643,
			0.219169,
			0.223719,
			0.228290,
			0.232883,
			0.237496,
			0.242129,
			0.246782,
			0.251452,
			0.256141,
			0.260846,
			0.265567,
			0.270303,
			0.275054,
			0.279819,
			0.284597,
			0.289387,
			0.294189,
			0.299002,
			0.303825,
			0.308658,
			0.313500,
			0.318350,
			0.323207,
			0.328072,
			0.332942,
			0.337818,
			0.342699,
			0.347584,
			0.352473,
			0.357365,
			0.362259,
			0.367155,
			0.372051,
			0.376949,
			0.381846,
			0.386742,
			0.391637,
			0.396531,
			0.401422,
			0.406309,
			0.411194,
			0.416074,
			0.420949,
			0.425819,
			0.430684,
			0.435542,
			0.440393,
			0.445237,
			0.450073,
			0.454901,
			0.459720,
			0.464530,
			0.469330,
			0.474120,
			0.478899,
			0.483667,
			0.488423,
			0.493167,
			0.497899,
			0.502618,
			0.507324,
			0.512015,
			0.516693,
			0.521356,
			0.526005,
			0.530638,
			0.535255,
			0.539856,
			0.544441,
			0.549009,
			0.553559,
			0.558093,
			0.562608,
			0.567106,
			0.571584,
			0.576044,
			0.580485,
			0.584907,
			0.589309,
			0.593691,
			0.598052,
			0.602393,
			0.606713,
			0.611013,
			0.615290,
			0.619546,
			0.623781,
			0.627993,
			0.632183,
			0.636350,
			0.640495,
			0.644616,
			0.648715,
			0.652789,
			0.656841,
			0.660868,
			0.664872,
			0.668852,
			0.672807,
			0.676737,
			0.680643,
			0.684525,
			0.688381,
			0.692212,
			0.696018,
			0.699799,
			0.703554,
			0.707283,
			0.710987,
			0.714665,
			0.718317,
			0.721942,
			0.725542,
			0.729115,
			0.732662,
			0.736183,
			0.739677,
			0.743145,
			0.746586,
			0.750000,
			0.753387,
			0.756748,
			0.760082,
			0.763389,
			0.766669,
			0.769922,
			0.773148,
			0.776347,
			0.779519,
			0.782663,
			0.785781,
			0.788872,
			0.791935,
			0.794972,
			0.797981,
			0.800963,
			0.803918,
			0.806846,
			0.809747,
			0.812621,
			0.815468,
			0.818288,
			0.821081,
			0.823847,
			0.826586,
			0.829299,
			0.831984,
			0.834643,
			0.837275,
			0.839881,
			0.842460,
			0.845012,
			0.847538,
			0.850037,
			0.852511,
			0.854958,
			0.857378,
			0.859773,
			0.862142,
			0.864484,
			0.866801,
			0.869092,
			0.871358,
			0.873598,
			0.875812,
			0.878001,
			0.880165,
			0.882303,
			0.884417,
			0.886505,
			0.888569,
			0.890608,
			0.892622,
			0.894612,
			0.896577,
			0.898518,
			0.900435,
			0.902328,
			0.904197,
			0.906042,
			0.907863,
			0.909661,
			0.911436,
			0.913187,
			0.914915,
			0.916620,
			0.918302,
			0.919962,
			0.921598,
			0.923213,
			0.924805,
			0.926375,
			0.927923,
			0.929449,
			0.930953,
			0.932436,
			0.933897,
			0.935337,
			0.936756,
			0.938153,
			0.939530,
			0.940887,
			0.942222,
			0.943538,
			0.944833,
			0.946108,
			0.947363,
			0.948598,
			0.949814,
			0.951010,
			0.952188,
			0.953345,
			0.954484,
			0.955605,
			0.956706,
			0.957789,
			0.958854,
			0.959900,
			0.960929,
			0.961940,
			0.962933,
			0.963908,
			0.964867,
			0.965808,
			0.966732,
			0.967639,
			0.968529,
			0.969403,
			0.970261,
			0.971102,
			0.971927,
			0.972737,
			0.973530,
			0.974308,
			0.975071,
			0.975819,
			0.976551,
			0.977269,
			0.977972,
			0.978660,
			0.979334,
			0.979993,
			0.980639,
			0.981270,
			0.981888,
			0.982492,
			0.983083,
			0.983660,
			0.984225,
			0.984776,
			0.985315,
			0.985841,
			0.986354,
			0.986855,
			0.987344,
			0.987821,
			0.988286,
			0.988739,
			0.989181,
			0.989611,
			0.990031,
			0.990439,
			0.990836,
			0.991222,
			0.991598,
			0.991963,
			0.992318,
			0.992662,
			0.992997,
			0.993322,
			0.993637,
			0.993942,
			0.994238,
			0.994525,
			0.994802,
			0.995071,
			0.995330,
			0.995581,
			0.995823,
			0.996057,
			0.996283,
			0.996500,
			0.996710,
			0.996911,
			0.997105,
			0.997291,
			0.997470,
			0.997641,
			0.997805,
			0.997962,
			0.998112,
			0.998255,
			0.998392,
			0.998522,
			0.998645,
			0.998762,
			0.998874,
			0.998979,
			0.999078,
			0.999171,
			0.999259,
			0.999341,
			0.999417,
			0.999488,
			0.999555,
			0.999616,
			0.999672,
			0.999723,
			0.999769,
			0.999811,
			0.999848,
			0.999881,
			0.999910,
			0.999935,
			0.999955,
			0.999971,
			0.999984,
			0.999993,
			0.999998,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			1.000000,
			0.999998,
			0.999993,
			0.999984,
			0.999971,
			0.999955,
			0.999935,
			0.999910,
			0.999881,
			0.999848,
			0.999811,
			0.999769,
			0.999723,
			0.999672,
			0.999616,
			0.999555,
			0.999488,
			0.999417,
			0.999341,
			0.999259,
			0.999171,
			0.999078,
			0.998979,
			0.998874,
			0.998762,
			0.998645,
			0.998522,
			0.998392,
			0.998255,
			0.998112,
			0.997962,
			0.997805,
			0.997641,
			0.997470,
			0.997291,
			0.997105,
			0.996911,
			0.996710,
			0.996500,
			0.996283,
			0.996057,
			0.995823,
			0.995581,
			0.995330,
			0.995071,
			0.994802,
			0.994525,
			0.994238,
			0.993942,
			0.993637,
			0.993322,
			0.992997,
			0.992662,
			0.992318,
			0.991963,
			0.991598,
			0.991222,
			0.990836,
			0.990439,
			0.990031,
			0.989611,
			0.989181,
			0.988739,
			0.988286,
			0.987821,
			0.987344,
			0.986855,
			0.986354,
			0.985841,
			0.985315,
			0.984776,
			0.984225,
			0.983660,
			0.983083,
			0.982492,
			0.981888,
			0.981270,
			0.980639,
			0.979993,
			0.979334,
			0.978660,
			0.977972,
			0.977269,
			0.976551,
			0.975819,
			0.975071,
			0.974308,
			0.973530,
			0.972737,
			0.971927,
			0.971102,
			0.970261,
			0.969403,
			0.968529,
			0.967639,
			0.966732,
			0.965808,
			0.964867,
			0.963908,
			0.962933,
			0.961940,
			0.960929,
			0.959900,
			0.958854,
			0.957789,
			0.956706,
			0.955605,
			0.954484,
			0.953345,
			0.952188,
			0.951010,
			0.949814,
			0.948598,
			0.947363,
			0.946108,
			0.944833,
			0.943538,
			0.942222,
			0.940887,
			0.939530,
			0.938153,
			0.936756,
			0.935337,
			0.933897,
			0.932436,
			0.930953,
			0.929449,
			0.927923,
			0.926375,
			0.924805,
			0.923213,
			0.921598,
			0.919962,
			0.918302,
			0.916620,
			0.914915,
			0.913187,
			0.911436,
			0.909661,
			0.907863,
			0.906042,
			0.904197,
			0.902328,
			0.900435,
			0.898518,
			0.896577,
			0.894612,
			0.892622,
			0.890608,
			0.888569,
			0.886505,
			0.884417,
			0.882303,
			0.880165,
			0.878001,
			0.875812,
			0.873598,
			0.871358,
			0.869092,
			0.866801,
			0.864484,
			0.862142,
			0.859773,
			0.857378,
			0.854958,
			0.852511,
			0.850037,
			0.847538,
			0.845012,
			0.842460,
			0.839881,
			0.837275,
			0.834643,
			0.831984,
			0.829299,
			0.826586,
			0.823847,
			0.821081,
			0.818288,
			0.815468,
			0.812621,
			0.809747,
			0.806846,
			0.803918,
			0.800963,
			0.797981,
			0.794972,
			0.791935,
			0.788872,
			0.785781,
			0.782663,
			0.779519,
			0.776347,
			0.773148,
			0.769922,
			0.766669,
			0.763389,
			0.760082,
			0.756748,
			0.753387,
			0.750000,
			0.746586,
			0.743145,
			0.739677,
			0.736183,
			0.732662,
			0.729115,
			0.725542,
			0.721942,
			0.718317,
			0.714665,
			0.710987,
			0.707283,
			0.703554,
			0.699799,
			0.696018,
			0.692212,
			0.688381,
			0.684525,
			0.680643,
			0.676737,
			0.672807,
			0.668852,
			0.664872,
			0.660868,
			0.656841,
			0.652789,
			0.648715,
			0.644616,
			0.640495,
			0.636350,
			0.632183,
			0.627993,
			0.623781,
			0.619546,
			0.615290,
			0.611013,
			0.606713,
			0.602393,
			0.598052,
			0.593691,
			0.589309,
			0.584907,
			0.580485,
			0.576044,
			0.571584,
			0.567106,
			0.562608,
			0.558093,
			0.553559,
			0.549009,
			0.544441,
			0.539856,
			0.535255,
			0.530638,
			0.526005,
			0.521356,
			0.516693,
			0.512015,
			0.507324,
			0.502618,
			0.497899,
			0.493167,
			0.488423,
			0.483667,
			0.478899,
			0.474120,
			0.469330,
			0.464530,
			0.459720,
			0.454901,
			0.450073,
			0.445237,
			0.440393,
			0.435542,
			0.430684,
			0.425819,
			0.420949,
			0.416074,
			0.411194,
			0.406309,
			0.401422,
			0.396531,
			0.391637,
			0.386742,
			0.381846,
			0.376949,
			0.372051,
			0.367155,
			0.362259,
			0.357365,
			0.352473,
			0.347584,
			0.342699,
			0.337818,
			0.332942,
			0.328072,
			0.323207,
			0.318350,
			0.313500,
			0.308658,
			0.303825,
			0.299002,
			0.294189,
			0.289387,
			0.284597,
			0.279819,
			0.275054,
			0.270303,
			0.265567,
			0.260846,
			0.256141,
			0.251452,
			0.246782,
			0.242129,
			0.237496,
			0.232883,
			0.228290,
			0.223719,
			0.219169,
			0.214643,
			0.210141,
			0.205663,
			0.201210,
			0.196784,
			0.192384,
			0.188012,
			0.183669,
			0.179356,
			0.175072,
			0.170819,
			0.166599,
			0.162411,
			0.158256,
			0.154136,
			0.150051,
			0.146003,
			0.141991,
			0.138016,
			0.134081,
			0.130184,
			0.126328,
			0.122514,
			0.118741,
			0.115011,
			0.111324,
			0.107682,
			0.104086,
			0.100535,
			0.097032,
			0.093577,
			0.090170,
			0.086813,
			0.083507,
			0.080252,
			0.077049,
			0.073899,
			0.070803,
			0.067762,
			0.064776,
			0.061847,
			0.058975,
			0.056161,
			0.053406,
			0.050710,
			0.048075,
			0.045502,
			0.042991,
			0.040542,
			0.038158,
			0.035838,
			0.033584,
			0.031396,
			0.029274,
			0.027221,
			0.025236,
			0.023320,
			0.021475,
			0.019700,
			0.017997,
			0.016366,
			0.014808,
			0.013324,
			0.011915,
			0.010581,
			0.009322,
			0.008140,
			0.007036,
			0.006009,
			0.005061,
			0.004193,
			0.003404,
			0.002695,
			0.002068,
			0.001523,
			0.001060,
			0.000680,
			0.000383,
			0.000171,
			0.000043,
			0.000000
};
const static float dot_table[1001]=
{
	0.000043,
	0.000128,
	0.000212,
	0.000297,
	0.00038,
	0.000463,
	0.000545,
	0.000627,
	0.000709,
	0.000789,
	0.000868,
	0.000948,
	0.001027,
	0.001104,
	0.001182,
	0.001259,
	0.001334,
	0.001409,
	0.001484,
	0.001558,
	0.001631,
	0.001703,
	0.001775,
	0.001845,
	0.001916,
	0.001985,
	0.002053,
	0.002122,
	0.002188,
	0.002254,
	0.00232,
	0.002384,
	0.002449,
	0.002511,
	0.002573,
	0.002635,
	0.002696,
	0.002755,
	0.002814,
	0.002872,
	0.002929,
	0.002986,
	0.003041,
	0.003096,
	0.00315,
	0.003203,
	0.003255,
	0.003306,
	0.003357,
	0.003407,
	0.003455,
	0.003503,
	0.003551,
	0.003596,
	0.003642,
	0.003687,
	0.00373,
	0.003773,
	0.003814,
	0.003856,
	0.003897,
	0.003935,
	0.003975,
	0.004012,
	0.004048,
	0.004085,
	0.00412,
	0.004155,
	0.004188,
	0.00422,
	0.004253,
	0.004284,
	0.004313,
	0.004343,
	0.004372,
	0.0044,
	0.004426,
	0.004453,
	0.004478,
	0.004502,
	0.004526,
	0.00455,
	0.004571,
	0.004593,
	0.004613,
	0.004633,
	0.004653,
	0.00467,
	0.004689,
	0.004705,
	0.004721,
	0.004736,
	0.004751,
	0.004765,
	0.004778,
	0.00479,
	0.004802,
	0.004813,
	0.004823,
	0.004833,
	0.004842,
	0.00485,
	0.004857,
	0.004865,
	0.00487,
	0.004876,
	0.004881,
	0.004885,
	0.004889,
	0.004892,
	0.004894,
	0.004896,
	0.004896,
	0.004898,
	0.004897,
	0.004896,
	0.004895,
	0.004894,
	0.004891,
	0.004887,
	0.004885,
	0.00488,
	0.004875,
	0.00487,
	0.004865,
	0.004858,
	0.004851,
	0.004844,
	0.004836,
	0.004828,
	0.004819,
	0.00481,
	0.0048,
	0.00479,
	0.004779,
	0.004768,
	0.004756,
	0.004744,
	0.004732,
	0.004719,
	0.004706,
	0.004691,
	0.004678,
	0.004663,
	0.004649,
	0.004633,
	0.004617,
	0.004601,
	0.004585,
	0.004568,
	0.00455,
	0.004534,
	0.004515,
	0.004498,
	0.004478,
	0.00446,
	0.004441,
	0.004422,
	0.004402,
	0.004382,
	0.004361,
	0.004341,
	0.00432,
	0.0043,
	0.004277,
	0.004256,
	0.004235,
	0.004212,
	0.00419,
	0.004167,
	0.004145,
	0.004121,
	0.004099,
	0.004074,
	0.004052,
	0.004027,
	0.004004,
	0.00398,
	0.003955,
	0.00393,
	0.003906,
	0.003882,
	0.003856,
	0.003831,
	0.003806,
	0.003781,
	0.003755,
	0.003729,
	0.003704,
	0.003678,
	0.003652,
	0.003625,
	0.0036,
	0.003573,
	0.003547,
	0.003521,
	0.003494,
	0.003468,
	0.003441,
	0.003414,
	0.003387,
	0.003361,
	0.003334,
	0.003307,
	0.00328,
	0.003253,
	0.003226,
	0.003199,
	0.003172,
	0.003144,
	0.003118,
	0.003091,
	0.003063,
	0.003037,
	0.003009,
	0.002982,
	0.002955,
	0.002928,
	0.002901,
	0.002874,
	0.002847,
	0.00282,
	0.002793,
	0.002766,
	0.002739,
	0.002713,
	0.002685,
	0.002659,
	0.002632,
	0.002606,
	0.002579,
	0.002552,
	0.002526,
	0.002499,
	0.002474,
	0.002447,
	0.00242,
	0.002395,
	0.002369,
	0.002342,
	0.002317,
	0.002291,
	0.002266,
	0.00224,
	0.002214,
	0.002189,
	0.002164,
	0.002138,
	0.002114,
	0.002088,
	0.002064,
	0.002039,
	0.002014,
	0.00199,
	0.001965,
	0.001941,
	0.001917,
	0.001893,
	0.001869,
	0.001845,
	0.001821,
	0.001798,
	0.001775,
	0.001751,
	0.001728,
	0.001705,
	0.001682,
	0.00166,
	0.001636,
	0.001615,
	0.001592,
	0.00157,
	0.001548,
	0.001526,
	0.001504,
	0.001483,
	0.001461,
	0.00144,
	0.001419,
	0.001397,
	0.001377,
	0.001357,
	0.001335,
	0.001316,
	0.001295,
	0.001275,
	0.001255,
	0.001235,
	0.001216,
	0.001196,
	0.001178,
	0.001157,
	0.001139,
	0.001121,
	0.001101,
	0.001083,
	0.001065,
	0.001046,
	0.001029,
	0.001011,
	0.000993,
	0.000975,
	0.000959,
	0.000941,
	0.000924,
	0.000907,
	0.00089,
	0.000874,
	0.000858,
	0.000841,
	0.000825,
	0.00081,
	0.000793,
	0.000778,
	0.000763,
	0.000748,
	0.000732,
	0.000718,
	0.000703,
	0.000688,
	0.000674,
	0.000659,
	0.000646,
	0.000631,
	0.000618,
	0.000604,
	0.000591,
	0.000577,
	0.000565,
	0.000551,
	0.000539,
	0.000526,
	0.000513,
	0.000501,
	0.000489,
	0.000477,
	0.000465,
	0.000453,
	0.000442,
	0.00043,
	0.00042,
	0.000408,
	0.000397,
	0.000386,
	0.000376,
	0.000365,
	0.000355,
	0.000344,
	0.000335,
	0.000325,
	0.000315,
	0.000305,
	0.000296,
	0.000287,
	0.000277,
	0.000269,
	0.000259,
	0.000251,
	0.000242,
	0.000234,
	0.000226,
	0.000217,
	0.00021,
	0.000201,
	0.000194,
	0.000186,
	0.000179,
	0.000171,
	0.000164,
	0.000157,
	0.00015,
	0.000143,
	0.000137,
	0.00013,
	0.000123,
	0.000117,
	0.000112,
	0.000105,
	9.9E-05,
	9.3E-05,
	8.8E-05,
	8.2E-05,
	7.6E-05,
	7.1E-05,
	6.7E-05,
	6.1E-05,
	5.6E-05,
	5.1E-05,
	4.6E-05,
	4.2E-05,
	3.7E-05,
	3.3E-05,
	2.9E-05,
	2.5E-05,
	2E-05,
	1.6E-05,
	1.3E-05,
	9E-06,
	5E-06,
	2E-06,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	-2E-06,
	-5E-06,
	-9E-06,
	-1.3E-05,
	-1.6E-05,
	-2E-05,
	-2.5E-05,
	-2.9E-05,
	-3.3E-05,
	-3.7E-05,
	-4.2E-05,
	-4.6E-05,
	-5.1E-05,
	-5.6E-05,
	-6.1E-05,
	-6.7E-05,
	-7.1E-05,
	-7.6E-05,
	-8.2E-05,
	-8.8E-05,
	-9.3E-05,
	-9.9E-05,
	-0.000105,
	-0.000112,
	-0.000117,
	-0.000123,
	-0.00013,
	-0.000137,
	-0.000143,
	-0.00015,
	-0.000157,
	-0.000164,
	-0.000171,
	-0.000179,
	-0.000186,
	-0.000194,
	-0.000201,
	-0.00021,
	-0.000217,
	-0.000226,
	-0.000234,
	-0.000242,
	-0.000251,
	-0.000259,
	-0.000269,
	-0.000277,
	-0.000287,
	-0.000296,
	-0.000305,
	-0.000315,
	-0.000325,
	-0.000335,
	-0.000344,
	-0.000355,
	-0.000365,
	-0.000376,
	-0.000386,
	-0.000397,
	-0.000408,
	-0.00042,
	-0.00043,
	-0.000442,
	-0.000453,
	-0.000465,
	-0.000477,
	-0.000489,
	-0.000501,
	-0.000513,
	-0.000526,
	-0.000539,
	-0.000551,
	-0.000565,
	-0.000577,
	-0.000591,
	-0.000604,
	-0.000618,
	-0.000631,
	-0.000646,
	-0.000659,
	-0.000674,
	-0.000688,
	-0.000703,
	-0.000718,
	-0.000732,
	-0.000748,
	-0.000763,
	-0.000778,
	-0.000793,
	-0.00081,
	-0.000825,
	-0.000841,
	-0.000858,
	-0.000874,
	-0.00089,
	-0.000907,
	-0.000924,
	-0.000941,
	-0.000959,
	-0.000975,
	-0.000993,
	-0.001011,
	-0.001029,
	-0.001046,
	-0.001065,
	-0.001083,
	-0.001101,
	-0.001121,
	-0.001139,
	-0.001157,
	-0.001178,
	-0.001196,
	-0.001216,
	-0.001235,
	-0.001255,
	-0.001275,
	-0.001295,
	-0.001316,
	-0.001335,
	-0.001357,
	-0.001377,
	-0.001397,
	-0.001419,
	-0.00144,
	-0.001461,
	-0.001483,
	-0.001504,
	-0.001526,
	-0.001548,
	-0.00157,
	-0.001592,
	-0.001615,
	-0.001636,
	-0.00166,
	-0.001682,
	-0.001705,
	-0.001728,
	-0.001751,
	-0.001775,
	-0.001798,
	-0.001821,
	-0.001845,
	-0.001869,
	-0.001893,
	-0.001917,
	-0.001941,
	-0.001965,
	-0.00199,
	-0.002014,
	-0.002039,
	-0.002064,
	-0.002088,
	-0.002114,
	-0.002138,
	-0.002164,
	-0.002189,
	-0.002214,
	-0.00224,
	-0.002266,
	-0.002291,
	-0.002317,
	-0.002342,
	-0.002369,
	-0.002395,
	-0.00242,
	-0.002447,
	-0.002474,
	-0.002499,
	-0.002526,
	-0.002552,
	-0.002579,
	-0.002606,
	-0.002632,
	-0.002659,
	-0.002685,
	-0.002713,
	-0.002739,
	-0.002766,
	-0.002793,
	-0.00282,
	-0.002847,
	-0.002874,
	-0.002901,
	-0.002928,
	-0.002955,
	-0.002982,
	-0.003009,
	-0.003037,
	-0.003063,
	-0.003091,
	-0.003118,
	-0.003144,
	-0.003172,
	-0.003199,
	-0.003226,
	-0.003253,
	-0.00328,
	-0.003307,
	-0.003334,
	-0.003361,
	-0.003387,
	-0.003414,
	-0.003441,
	-0.003468,
	-0.003494,
	-0.003521,
	-0.003547,
	-0.003573,
	-0.0036,
	-0.003625,
	-0.003652,
	-0.003678,
	-0.003704,
	-0.003729,
	-0.003755,
	-0.003781,
	-0.003806,
	-0.003831,
	-0.003856,
	-0.003882,
	-0.003906,
	-0.00393,
	-0.003955,
	-0.00398,
	-0.004004,
	-0.004027,
	-0.004052,
	-0.004074,
	-0.004099,
	-0.004121,
	-0.004145,
	-0.004167,
	-0.00419,
	-0.004212,
	-0.004235,
	-0.004256,
	-0.004277,
	-0.0043,
	-0.00432,
	-0.004341,
	-0.004361,
	-0.004382,
	-0.004402,
	-0.004422,
	-0.004441,
	-0.00446,
	-0.004478,
	-0.004498,
	-0.004515,
	-0.004534,
	-0.00455,
	-0.004568,
	-0.004585,
	-0.004601,
	-0.004617,
	-0.004633,
	-0.004649,
	-0.004663,
	-0.004678,
	-0.004691,
	-0.004706,
	-0.004719,
	-0.004732,
	-0.004744,
	-0.004756,
	-0.004768,
	-0.004779,
	-0.00479,
	-0.0048,
	-0.00481,
	-0.004819,
	-0.004828,
	-0.004836,
	-0.004844,
	-0.004851,
	-0.004858,
	-0.004865,
	-0.00487,
	-0.004875,
	-0.00488,
	-0.004885,
	-0.004887,
	-0.004891,
	-0.004894,
	-0.004895,
	-0.004896,
	-0.004897,
	-0.004898,
	-0.004896,
	-0.004896,
	-0.004894,
	-0.004892,
	-0.004889,
	-0.004885,
	-0.004881,
	-0.004876,
	-0.00487,
	-0.004865,
	-0.004857,
	-0.00485,
	-0.004842,
	-0.004833,
	-0.004823,
	-0.004813,
	-0.004802,
	-0.00479,
	-0.004778,
	-0.004765,
	-0.004751,
	-0.004736,
	-0.004721,
	-0.004705,
	-0.004689,
	-0.00467,
	-0.004653,
	-0.004633,
	-0.004613,
	-0.004593,
	-0.004571,
	-0.00455,
	-0.004526,
	-0.004502,
	-0.004478,
	-0.004453,
	-0.004426,
	-0.0044,
	-0.004372,
	-0.004343,
	-0.004313,
	-0.004284,
	-0.004253,
	-0.00422,
	-0.004188,
	-0.004155,
	-0.00412,
	-0.004085,
	-0.004048,
	-0.004012,
	-0.003975,
	-0.003935,
	-0.003897,
	-0.003856,
	-0.003814,
	-0.003773,
	-0.00373,
	-0.003687,
	-0.003642,
	-0.003596,
	-0.003551,
	-0.003503,
	-0.003455,
	-0.003407,
	-0.003357,
	-0.003306,
	-0.003255,
	-0.003203,
	-0.00315,
	-0.003096,
	-0.003041,
	-0.002986,
	-0.002929,
	-0.002872,
	-0.002814,
	-0.002755,
	-0.002696,
	-0.002635,
	-0.002573,
	-0.002511,
	-0.002449,
	-0.002384,
	-0.00232,
	-0.002254,
	-0.002188,
	-0.002122,
	-0.002053,
	-0.001985,
	-0.001916,
	-0.001845,
	-0.001775,
	-0.001703,
	-0.001631,
	-0.001558,
	-0.001484,
	-0.001409,
	-0.001334,
	-0.001259,
	-0.001182,
	-0.001104,
	-0.001027,
	-0.000948,
	-0.000868,
	-0.000789,
	-0.000709,
	-0.000627,
	-0.000545,
	-0.000463,
	-0.00038,
	-0.000297,
	-0.000212,
	-0.000128,
	-0.000043,
	0

};

const static float accel_Integral = 0.7043;



#endif /* CPP_INC_TURN_TABLE_H_ */
