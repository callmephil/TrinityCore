-- 
DELETE FROM `gameobject` WHERE `guid` IN (8687, 8688, 8693, 8697, 8698, 8699, 8711, 8715, 8716, 8719, 8740, 8742, 8743, 8744, 8747, 8749, 8752, 8753, 8754, 8769, 8771, 8777, 8779, 8780, 8781, 8782, 8783, 8784, 8785, 8786, 8787, 8788, 8789, 8790, 8792, 8798, 8803, 8806, 8818, 8819, 8820, 8827, 8830, 8833, 8840, 8847, 8848, 8849, 8850, 8851, 8855, 8858, 8863, 8867, 8870, 8871, 8873, 8875, 8878, 8885, 8892, 8897, 8907, 8908, 8909, 8912, 8925, 8926, 8927, 8934, 8939, 8946);
INSERT INTO `gameobject` (`guid`, `id`, `map`, `spawnMask`, `phaseMask`, `position_x`, `position_y`, `position_z`, `orientation`, `rotation0`, `rotation1`, `rotation2`, `rotation3`, `spawntimesecs`, `animprogress`, `state`) VALUES
(8687,186459,571,1,1,1913.46,-4035.634,215.9161,0.4712385,0,0,0.2334452,0.97237,-1,255,1), -- Skorn Tower NW Bunny
(8688,186459,571,1,1,1903.368,-4030.903,220.6885,5.009095,0,0,-0.5948229,0.8038568,-1,255,1),
(8693,186459,571,1,1,1923.009,-4040.563,220.726,5.183629,0,0,-0.5224981,0.8526405,-1,255,1),
(8697,186459,571,1,1,1923.54,-4042.184,251.9819,0.2617982,0,0,0.1305256,0.9914449,-1,255,1),
(8698,186459,571,1,1,1906.085,-4047.991,252.1254,3.735006,0,0,-0.9563046,0.2923723,-1,255,1),
(8699,186459,571,1,1,1899.642,-4032.766,251.8167,4.206246,0,0,-0.8616285,0.5075394,-1,255,1),
(8711,186459,571,1,1,1915.125,-4026.924,256.8756,3.630291,0,0,-0.970295,0.241925,-1,255,1),
(8715,186459,571,1,1,1903.563,-4035.313,265.3723,2.792518,0,0,0.984807,0.1736523,-1,255,1),
(8716,186459,571,1,1,1917.825,-4041.076,265.8944,2.530723,0,0,0.9537163,0.3007079,-1,255,1),
(8719,186457,571,1,1,1913.432,-4035.679,215.9161,5.724681,0,0,-0.2756367,0.9612619,-1,255,1),
(8740,186457,571,1,1,1903.184,-4030.939,220.695,1.500983,0,0,0.6819983,0.7313538,-1,255,1),
(8742,186457,571,1,1,1923.122,-4040.392,220.7229,0.6283169,0,0,0.3090162,0.9510568,-1,255,1),
(8743,186457,571,1,1,1924.425,-4042.255,252.033,2.426008,0,0,0.9366722,0.3502074,-1,255,1),
(8744,186457,571,1,1,1905.783,-4047.757,252.1259,1.588249,0,0,0.7132502,0.7009096,-1,255,1),
(8747,186457,571,1,1,1899.019,-4033.115,251.8324,3.368496,0,0,-0.9935713,0.1132084,-1,255,1),
(8749,186457,571,1,1,1915.254,-4026.833,256.8428,5.794494,0,0,-0.2419214,0.9702958,-1,255,1),
(8752,186457,571,1,1,1903.396,-4035.186,265.2672,5.218536,0,0,-0.5075378,0.8616294,-1,255,1),
(8753,186457,571,1,1,1917.793,-4040.472,266.105,3.944446,0,0,-0.9205046,0.3907318,-1,255,1),
(8754,186459,571,1,1,1627.913,-4239.781,301.2891,4.921829,0,0,-0.6293201,0.7771462,-1,255,1),-- Skorn Tower SE Bunny
(8769,186459,571,1,1,1636.339,-4251.719,300.5544,5.567601,0,0,-0.3502073,0.9366722,-1,255,1),
(8771,186459,571,1,1,1621.623,-4251.325,291.8906,4.520406,0,0,-0.7716236,0.6360794,-1,255,1),
(8777,186459,571,1,1,1638.238,-4256.573,286.8297,0.7330382,0,0,0.3583679,0.9335805,-1,255,1),
(8779,186459,571,1,1,1642.142,-4241.497,287.1169,3.159062,0,0,-0.9999619,0.008734641,-1,255,1),
(8780,186459,571,1,1,1623.181,-4235.038,286.9467,3.543024,0,0,-0.9799242,0.1993704,-1,255,1),
(8781,186459,571,1,1,1624.175,-4236.356,255.6928,1.815142,0,0,0.7880106,0.6156617,-1,255,1),
(8782,186459,571,1,1,1633.752,-4256.174,255.6973,2.303831,0,0,0.9135447,0.4067384,-1,255,1),
(8783,186459,571,1,1,1628.448,-4246.229,250.9049,1.745327,0,0,0.7660437,0.6427886,-1,255,1),
(8784,186457,571,1,1,1628.439,-4246.249,250.9049,4.76475,0,0,-0.6883545,0.7253745,-1,255,1),
(8785,186457,571,1,1,1633.663,-4256.241,255.7013,6.230826,0,0,-0.02617645,0.9996573,-1,255,1),
(8786,186457,571,1,1,1624.156,-4236.16,255.7065,5.602507,0,0,-0.333807,0.9426414,-1,255,1),
(8787,186457,571,1,1,1623.238,-4234.662,286.9728,4.223697,0,0,-0.8571672,0.5150382,-1,255,1),
(8788,186457,571,1,1,1627.938,-4239.604,301.1777,4.153885,0,0,-0.8746195,0.4848101,-1,255,1),
(8789,186457,571,1,1,1636.363,-4251.83,300.498,5.532695,0,0,-0.3665009,0.9304177,-1,255,1),
(8790,186457,571,1,1,1621.609,-4251.186,291.8821,5.358162,0,0,-0.4461975,0.8949345,-1,255,1),
(8792,186457,571,1,1,1638.238,-4256.573,286.8297,4.01426,0,0,-0.9063072,0.4226195,-1,255,1),
(8798,186457,571,1,1,1642.384,-4241.351,287.1244,1.605702,0,0,0.7193394,0.6946588,-1,255,1),
(8803,186459,571,1,1,1657.958,-4003.128,249.7221,1.06465,0,0,0.5075378,0.8616294,-1,255,1),-- Skorn Tower SW Bunny
(8806,186459,571,1,1,1649.182,-4010.967,254.5305,3.298687,0,0,-0.9969168,0.07846643,-1,255,1),
(8818,186459,571,1,1,1666.674,-3996.285,254.5412,4.01426,0,0,-0.9063072,0.4226195,-1,255,1),
(8819,186459,571,1,1,1670.432,-3995.889,285.9533,2.722713,0,0,0.9781475,0.2079121,-1,255,1),
(8820,186459,571,1,1,1665.738,-4015.229,285.9299,0.7155849,0,0,0.3502073,0.9366722,-1,255,1),
(8827,186459,571,1,1,1649.672,-4015.667,285.6511,2.059488,0,0,0.8571672,0.5150382,-1,255,1),
(8830,186459,571,1,1,1650.934,-3997.695,290.6391,4.555311,0,0,-0.7604055,0.6494485,-1,255,1),
(8833,186459,571,1,1,1654.069,-4011.443,299.5486,2.548179,0,0,0.9563046,0.2923723,-1,255,1),
(8840,186459,571,1,1,1664.84,-4002.075,299.9129,2.705255,0,0,0.9762955,0.2164421,-1,255,1),
(8847,186457,571,1,1,1657.969,-4003.167,249.7221,2.007128,0,0,0.8433914,0.5372996,-1,255,1),
(8848,186457,571,1,1,1649.241,-4010.868,254.5244,5.358162,0,0,-0.4461975,0.8949345,-1,255,1),
(8849,186457,571,1,1,1666.658,-3996.389,254.5374,2.932139,0,0,0.9945211,0.1045355,-1,255,1),
(8850,186457,571,1,1,1670.648,-3995.774,285.9709,4.537859,0,0,-0.7660437,0.6427886,-1,255,1),
(8851,186457,571,1,1,1665.781,-4015.285,285.9318,1.308995,0,0,0.6087608,0.7933538,-1,255,1),
(8855,186457,571,1,1,1649.601,-4015.839,285.6599,5.550147,0,0,-0.3583679,0.9335805,-1,255,1),
(8858,186457,571,1,1,1650.877,-3997.604,290.5309,5.497789,0,0,-0.3826828,0.9238798,-1,255,1),
(8863,186457,571,1,1,1653.939,-4011.417,299.5074,0.01745246,0,0,0.00872612,0.9999619,-1,255,1),
(8867,186457,571,1,1,1664.91,-4002.066,299.8745,3.455756,0,0,-0.9876881,0.1564362,-1,255,1),
(8870,186459,571,1,1,1795.081,-4224.204,240.6801,3.647741,0,0,-0.9681473,0.2503814,-1,255,1),-- Skorn Tower E Bunny
(8871,186459,571,1,1,1802.785,-4216.037,246.937,5.811947,0,0,-0.2334452,0.97237,-1,255,1),
(8873,186459,571,1,1,1787.536,-4231.573,246.9303,0.6283169,0,0,0.3090162,0.9510568,-1,255,1),
(8875,186459,571,1,1,1786.043,-4213.064,278.3624,0.8203033,0,0,0.3987484,0.9170604,-1,255,1),
(8878,186459,571,1,1,1802.408,-4228.943,283.1333,3.595379,0,0,-0.97437,0.2249513,-1,255,1),
(8885,186459,571,1,1,1798.913,-4214.453,291.1866,0.9599299,0,0,0.4617481,0.8870111,-1,255,1),
(8892,186459,571,1,1,1788.069,-4225.396,292.4455,6.178466,0,0,-0.05233574,0.9986296,-1,255,1),
(8897,186459,571,1,1,1787.443,-4233.158,278.1396,2.024579,0,0,0.8480473,0.5299206,-1,255,1),
(8907,186459,571,1,1,1804.957,-4213.722,277.9689,6.230826,0,0,-0.02617645,0.9996573,-1,255,1),
(8908,186457,571,1,1,1795.081,-4224.204,240.6801,3.647741,0,0,-0.9681473,0.2503814,-1,255,1),
(8909,186457,571,1,1,1802.785,-4216.037,246.937,5.811947,0,0,-0.2334452,0.97237,-1,255,1),
(8912,186457,571,1,1,1787.536,-4231.573,246.9303,0.6283169,0,0,0.3090162,0.9510568,-1,255,1),
(8925,186457,571,1,1,1786.043,-4213.064,278.3624,0.8203033,0,0,0.3987484,0.9170604,-1,255,1),
(8926,186457,571,1,1,1802.408,-4228.943,283.1333,3.595379,0,0,-0.97437,0.2249513,-1,255,1),
(8927,186457,571,1,1,1798.913,-4214.453,291.1866,0.9599299,0,0,0.4617481,0.8870111,-1,255,1),
(8934,186457,571,1,1,1788.069,-4225.396,292.4455,6.178466,0,0,-0.05233574,0.9986296,-1,255,1),
(8939,186457,571,1,1,1787.443,-4233.158,278.1396,2.024579,0,0,0.8480473,0.5299206,-1,255,1),
(8946,186457,571,1,1,1804.957,-4213.722,277.9689,6.230826,0,0,-0.02617645,0.9996573,-1,255,1);
UPDATE `creature_template` SET `AIName` = 'SmartAI' WHERE `entry` = 24132;
DELETE FROM `smart_scripts`   WHERE `entryorguid` IN (24087, 24092, 24093, 24132, 24094, -103931, -104046, -104049, -104050, -104055, -104095, -104097, -104103) AND `source_type`=0;
DELETE FROM `smart_scripts`   WHERE `entryorguid` IN (2408700,2408701,2409200,2409300,2409400) AND `source_type`=9;
INSERT INTO `smart_scripts` (`entryorguid`, `source_type`, `id`, `link`, `event_type`, `event_phase_mask`, `event_chance`, `event_flags`, `event_param1`, `event_param2`, `event_param3`, `event_param4`, `action_type`, `action_param1`, `action_param2`, `action_param3`, `action_param4`, `action_param5`, `action_param6`, `target_type`, `target_param1`, `target_param2`, `target_param3`, `target_x`, `target_y`, `target_z`, `target_o`, `comment`) VALUES
(24087, 0, 0, 1, 8, 0, 100, 0, 49625, 0, 15000, 15000, 11, 43067, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower NW Bunny - On Spellhit 'Brave's Flare' - Cast 'Towers of Certain Doom: NW Kill Credit'"),
(24087, 0, 1, 0, 61, 0, 100, 512, 0, 0, 0, 0, 80, 2408701, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower NW Bunny - On Spellhit 'Sergeant's Flare' - Run Script"),
(24087, 0, 2, 3, 8, 0, 100, 0, 49634, 0, 15000, 15000, 11, 43067, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower NW Bunny - On Spellhit 'Sergeant's Flare' - Cast 'Towers of Certain Doom: NW Kill Credit'"),
(24087, 0, 3, 0, 61, 0, 100, 512, 0, 0, 0, 0, 80, 2408700, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower NW Bunny - On Spellhit 'Sergeant's Flare' - Run Script"),
(24092, 0, 0, 1, 8, 0, 100, 0, 49625, 0, 15000, 15000, 11, 43077, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower E Bunny - On Spellhit 'Brave's Flare' - Cast 'Towers of Certain Doom: E Kill Credit'"),
(24092, 0, 1, 0, 61, 0, 100, 512, 0, 0, 0, 0, 80, 2408701, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower E Bunny - On Spellhit 'Sergeant's Flare' - Run Script"),
(24092, 0, 2, 3, 8, 0, 100, 0, 49634, 0, 15000, 15000, 11, 43077, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower E Bunny - On Spellhit 'Sergeant's Flare' - Cast 'Towers of Certain Doom: E Kill Credit'"),
(24092, 0, 3, 0, 61, 0, 100, 512, 0, 0, 0, 0, 80, 2408700, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower E Bunny - On Spellhit 'Sergeant's Flare' - Run Script"),
(24093, 0, 0, 1, 8, 0, 100, 0, 49625, 0, 15000, 15000, 11, 43086, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower SW Bunny - On Spellhit 'Brave's Flare' - Cast 'Towers of Certain Doom: SW Kill Credit'"),
(24093, 0, 1, 0, 61, 0, 100, 512, 0, 0, 0, 0, 80, 2408701, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower SW Bunny - On Spellhit 'Sergeant's Flare' - Run Script"),
(24093, 0, 2, 3, 8, 0, 100, 0, 49634, 0, 15000, 15000, 11, 43086, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower SW Bunny - On Spellhit 'Sergeant's Flare' - Cast 'Towers of Certain Doom: SW Kill Credit'"),
(24093, 0, 3, 0, 61, 0, 100, 512, 0, 0, 0, 0, 80, 2408700, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower SW Bunny - On Spellhit 'Sergeant's Flare' - Run Script"),
(24094, 0, 0, 1, 8, 0, 100, 0, 49625, 0, 15000, 15000, 11, 43087, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower SE Bunny - On Spellhit 'Brave's Flare' - Cast 'Towers of Certain Doom: SE Kill Credit'"),
(24094, 0, 1, 0, 61, 0, 100, 512, 0, 0, 0, 0, 80, 2408701, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower SE Bunny - On Spellhit 'Sergeant's Flare' - Run Script"),
(24094, 0, 2, 3, 8, 0, 100, 0, 49634, 0, 15000, 15000, 11, 43087, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower SE Bunny - On Spellhit 'Sergeant's Flare' - Cast 'Towers of Certain Doom: SE Kill Credit'"),
(24094, 0, 3, 0, 61, 0, 100, 512, 0, 0, 0, 0, 80, 2408700, 2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower SE Bunny - On Spellhit 'Sergeant's Flare' - Run Script"),
(2408700, 9, 0, 0, 0, 0, 100, 0, 2000, 2000, 0, 0, 11, 56511, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Cast 'Towers of Certain Doom: Tower Bunny Smoke Flare Effect'"),
(2408700, 9, 1, 0, 0, 0, 100, 0, 15000, 15000, 0, 0, 11, 43069, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Cast 'Towers of Certain Doom: Skorn Cannonfire'"),
(2408700, 9, 2, 0, 0, 0, 100, 0, 100, 100, 0, 0, 11, 43072, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Invoker Cast 'Towers of Certain Doom: Tower Caster Instakill'"),
(2408700, 9, 3, 0, 0, 0, 100, 0, 0, 0, 0, 0, 70, 30, 0, 0, 0, 0, 0, 15, 186457, 35, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Respawn game object'"),
(2408700, 9, 4, 0, 0, 0, 100, 0, 0, 0, 0, 0, 70, 30, 0, 0, 0, 0, 0, 15, 186459, 35, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Respawn game object'"),
(2408701, 9, 0, 0, 0, 0, 100, 0, 2000, 2000, 0, 0, 11, 56511, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Cast 'Towers of Certain Doom: Tower Bunny Smoke Flare Effect'"),
(2408701, 9, 1, 0, 0, 0, 100, 0, 1000, 1000, 0, 0, 45, 1, 1, 0, 0, 0, 0, 11, 24132, 80, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Set Data'"),
(2408701, 9, 2, 0, 0, 0, 100, 0, 15000, 15000, 0, 0, 11, 43072, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Invoker Cast 'Towers of Certain Doom: Tower Caster Instakill'"),
(2408701, 9, 3, 0, 0, 0, 100, 0, 0, 0, 0, 0, 70, 30, 0, 0, 0, 0, 0, 15, 186457, 35, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Respawn game object'"),
(2408701, 9, 4, 0, 0, 0, 100, 0, 0, 0, 0, 0, 70, 30, 0, 0, 0, 0, 0, 15, 186459, 35, 0, 0, 0, 0, 0, "Skorn Tower Bunny - On Script - Respawn game object'"),
(-103931, 0, 0, 1, 11, 0, 100, 0, 0, 0, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider- On respawn - Set invisible"),
(-103931, 0, 1, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On respawn - Set event phase"),
(-103931, 0, 2, 3, 38, 0, 100, 0, 1, 1, 35000, 35000, 53, 1, 241324, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Start way point"),
(-103931, 0, 3, 4, 61, 0, 100, 0, 0, 0, 0, 0, 22, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set event phase"),
(-103931, 0, 4, 0, 61, 0, 100, 0, 0, 0, 0, 0, 47, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set visible"),
(-103931, 0, 5, 0, 1, 2, 100, 0, 1000, 1000, 2000, 3000, 11, 43109, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - Ooc - Cast 'Throw torch"),
(-103931, 0, 6, 7, 58, 0, 100, 0, 6, 241324, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set invisible"),
(-103931, 0, 7, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set event phase"),
(-104046, 0, 0, 1, 11, 0, 100, 0, 0, 0, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider- On respawn - Set invisible"),
(-104046, 0, 1, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On respawn - Set event phase"),
(-104046, 0, 2, 3, 38, 0, 100, 0, 1, 1, 35000, 35000, 53, 1, 241325, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Start way point"),
(-104046, 0, 3, 4, 61, 0, 100, 0, 0, 0, 0, 0, 22, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set event phase"),
(-104046, 0, 4, 0, 61, 0, 100, 0, 0, 0, 0, 0, 47, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set visible"),
(-104046, 0, 5, 0, 1, 2, 100, 0, 1000, 1000, 2000, 3000, 11, 43109, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - Ooc - Cast 'Throw torch"),
(-104046, 0, 6, 7, 58, 0, 100, 0, 6, 241325, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set invisible"),
(-104046, 0, 7, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set event phase"),
(-104049, 0, 0, 1, 11, 0, 100, 0, 0, 0, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider- On respawn - Set invisible"),
(-104049, 0, 1, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On respawn - Set event phase"),
(-104049, 0, 2, 3, 38, 0, 100, 0, 1, 1, 35000, 35000, 53, 1, 241326, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Start way point"),
(-104049, 0, 3, 4, 61, 0, 100, 0, 0, 0, 0, 0, 22, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set event phase"),
(-104049, 0, 4, 0, 61, 0, 100, 0, 0, 0, 0, 0, 47, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set visible"),
(-104049, 0, 5, 0, 1, 2, 100, 0, 1000, 1000, 2000, 3000, 11, 43109, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - Ooc - Cast 'Throw torch"),
(-104049, 0, 6, 7, 58, 0, 100, 0, 6, 241326, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set invisible"),
(-104049, 0, 7, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set event phase"),
(-104050, 0, 0, 1, 11, 0, 100, 0, 0, 0, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider- On respawn - Set invisible"),
(-104050, 0, 1, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On respawn - Set event phase"),
(-104050, 0, 2, 3, 38, 0, 100, 0, 1, 1, 35000, 35000, 53, 1, 241327, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Start way point"),
(-104050, 0, 3, 4, 61, 0, 100, 0, 0, 0, 0, 0, 22, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set event phase"),
(-104050, 0, 4, 0, 61, 0, 100, 0, 0, 0, 0, 0, 47, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set visible"),
(-104050, 0, 5, 0, 1, 2, 100, 0, 1000, 1000, 2000, 3000, 11, 43109, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - Ooc - Cast 'Throw torch"),
(-104050, 0, 6, 7, 58, 0, 100, 0, 6, 241327, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set invisible"),
(-104050, 0, 7, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set event phase"),
(-104055, 0, 0, 1, 11, 0, 100, 0, 0, 0, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider- On respawn - Set invisible"),
(-104055, 0, 1, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On respawn - Set event phase"),
(-104055, 0, 2, 3, 38, 0, 100, 0, 1, 1, 35000, 35000, 53, 1, 241322, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Start way point"),
(-104055, 0, 3, 4, 61, 0, 100, 0, 0, 0, 0, 0, 22, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set event phase"),
(-104055, 0, 4, 0, 61, 0, 100, 0, 0, 0, 0, 0, 47, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set visible"),
(-104055, 0, 5, 0, 1, 2, 100, 0, 1000, 1000, 2000, 3000, 11, 43109, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - Ooc - Cast 'Throw torch"),
(-104055, 0, 6, 7, 58, 0, 100, 0, 6, 241322, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set invisible"),
(-104055, 0, 7, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set event phase"),
(-104095, 0, 0, 1, 11, 0, 100, 0, 0, 0, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider- On respawn - Set invisible"),
(-104095, 0, 1, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On respawn - Set event phase"),
(-104095, 0, 2, 3, 38, 0, 100, 0, 1, 1, 35000, 35000, 53, 1, 241323, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Start way point"),
(-104095, 0, 3, 4, 61, 0, 100, 0, 0, 0, 0, 0, 22, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set event phase"),
(-104095, 0, 4, 0, 61, 0, 100, 0, 0, 0, 0, 0, 47, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set visible"),
(-104095, 0, 5, 0, 1, 2, 100, 0, 1000, 1000, 2000, 3000, 11, 43109, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - Ooc - Cast 'Throw torch"),
(-104095, 0, 6, 7, 58, 0, 100, 0, 6, 241323, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set invisible"),
(-104095, 0, 7, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set event phase"),
(-104097, 0, 0, 1, 11, 0, 100, 0, 0, 0, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider- On respawn - Set invisible"),
(-104097, 0, 1, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On respawn - Set event phase"),
(-104097, 0, 2, 3, 38, 0, 100, 0, 1, 1, 35000, 35000, 53, 1, 241320, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Start way point"),
(-104097, 0, 3, 4, 61, 0, 100, 0, 0, 0, 0, 0, 22, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set event phase"),
(-104097, 0, 4, 0, 61, 0, 100, 0, 0, 0, 0, 0, 47, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set visible"),
(-104097, 0, 5, 0, 1, 2, 100, 0, 1000, 1000, 2000, 3000, 11, 43109, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - Ooc - Cast 'Throw torch"),
(-104097, 0, 6, 7, 58, 0, 100, 0, 6, 241320, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set invisible"),
(-104097, 0, 7, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set event phase"),
(-104103, 0, 0, 1, 11, 0, 100, 0, 0, 0, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider- On respawn - Set invisible"),
(-104103, 0, 1, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On respawn - Set event phase"),
(-104103, 0, 2, 3, 38, 0, 100, 0, 1, 1, 35000, 35000, 53, 1, 241321, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Start way point"),
(-104103, 0, 3, 4, 61, 0, 100, 0, 0, 0, 0, 0, 22, 2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set event phase"),
(-104103, 0, 4, 0, 61, 0, 100, 0, 0, 0, 0, 0, 47, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On data set - Set visible"),
(-104103, 0, 5, 0, 1, 2, 100, 0, 1000, 1000, 2000, 3000, 11, 43109, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - Ooc - Cast 'Throw torch"),
(-104103, 0, 6, 7, 58, 0, 100, 0, 6, 241321, 0, 0, 47, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set invisible"),
(-104103, 0, 7, 0, 61, 0, 100, 0, 0, 0, 0, 0, 22, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, "Winterhoof Wind Rider - On waypoint ended - Set event phase");

DELETE FROM `creature` WHERE `guid` IN (103931, 104046, 104049, 104050, 104055, 104095, 104097, 104103);
INSERT INTO `creature` (`guid`, `id`, `map`, `zoneId`, `areaId`, `spawnMask`, `phaseMask`, `modelid`, `equipment_id`, `position_x`, `position_y`, `position_z`, `orientation`, `spawntimesecs`, `spawndist`, `currentwaypoint`, `curhealth`, `curmana`, `MovementType`, `npcflag`, `unit_flags`, `dynamicflags`, `VerifiedBuild`) VALUES 
(103931, 24132, 571, 0, 0, 1, 1, 0, 0, 1914.760010, -3967.179932, 286.723999,100, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(104046, 24132, 571, 0, 0, 1, 1, 0, 0, 1895.380005, -3970.090088, 288.824005,100, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(104049, 24132, 571, 0, 0, 1, 1, 0, 0, 1802.910034, -4171.540039, 293.82101,2.912611, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(104050, 24132, 571, 0, 0, 1, 1, 0, 0, 1816.680054, -4173.060059, 294.94101,2.498887, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(104055, 24132, 571, 0, 0, 1, 1, 0, 0, 1636.739990, -3951.48999, 305.75900, 1.86750, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0), 
(104095, 24132, 571, 0, 0, 1, 1, 0, 0, 1658.680054, -3941.48999, 307.50601, 1.51844, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0), 
(104097, 24132, 571, 0, 0, 1, 1, 0, 0, 1650.05004, -4194.359863, 310.941, 4.7822, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(104103, 24132, 571, 0, 0, 1, 1, 0, 0, 1633.04003, -4196.669922, 312.112, 3.1765, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0);

DELETE FROM `waypoints` WHERE `entry` IN (241320,241321,241322,241323,241324,241325,241326,241327);
INSERT INTO `waypoints` (`entry`, `pointid`, `position_x`, `position_y`, `position_z`) VALUES
-- #0
(241320,1,1665.1317, -4217.034668, 305.647888),
(241320,2,1661.6821, -4277.691406, 298.328064),
(241320,3,1624.9110, -4287.021484, 298.638702),
(241320,4,1587.1458, -4244.930176, 298.490875),
(241320,5,1605.9997, -4220.281250, 298.045197),
(241320,6,1650.0500, -4194.359863, 310.941010),
-- #1
(241321,1,1611.113647, -4207.125488, 310.43158),
(241321,2,1594.945801, -4221.966309, 306.13314),
(241321,3,1593.125732, -4258.428711, 301.94644),
(241321,4,1635.653076, -4286.640137, 302.76644),
(241321,5,1671.947266, -4254.366211, 297.35217),
(241321,6,1633.040039, -4196.669922, 312.11200),
-- #2
(241322,1,1626.73706, -3964.46215, 310.51687),
(241322,2,1611.14209, -4022.95971, 314.99606),
(241322,3,1654.66357, -4043.99194, 306.95568),
(241322,4,1693.23364, -4023.78125, 309.27948),
(241322,5,1684.92224, -3981.94018, 299.57269),
(241322,6,1636.73999, -3951.48999, 305.75900),
-- #3
(241323,1,1680.77307, -3969.44165, 302.98806),
(241323,2,1694.33166, -4004.84545, 297.83352),
(241323,3,1675.56115, -4044.35595, 297.29418),
(241323,4,1618.18249, -4027.03588, 291.66238),
(241323,5,1610.14404, -3987.65942, 299.26532),
(241323,6,1658.68005, -3941.48999, 307.50601),
-- #4
(241324,1,1929.052612, -3987.700439, 279.358002),
(241324,2,1941.117554, -4027.058838, 274.410645),
(241324,3,1927.333130, -4063.763916, 270.038269),
(241324,4,1894.148315, -4064.747559, 273.607819),
(241324,5,1879.348145, -4030.808105, 272.336426),
(241324,6,1914.760010, -3967.179932, 286.723999),
-- #5
(241325,1,1879.32116, -4022.1381, 273.5163),
(241325,2,1880.07739, -4049.3344, 271.6730),
(241325,3,1907.65234, -4074.2683, 272.6399),
(241325,4,1943.23791, -4053.4812, 266.2724),
(241325,5,1942.69030, -4022.8376, 272.0101),
(241325,6,1895.38000, -3970.0900, 288.8240),
-- #6
(241326,1,1767.899536, -4193.723633, 291.110931),
(241326,2,1750.262207, -4225.302734, 287.555237),
(241326,3,1768.416748, -4259.694336, 286.102783),
(241326,4,1797.169312, -4270.992676, 288.128418),
(241326,5,1837.245361, -4248.111328, 287.980835),
(241326,6,1802.910034, -4171.540031, 293.821014),
-- #7
(241327,1,1848.90625, -4217.54199, 292.86257),
(241327,2,1826.92480, -4260.11767, 285.04205),
(241327,3,1790.01586, -4266.68017, 281.37289),
(241327,4,1751.16027, -4225.78320, 284.94622),
(241327,5,1767.89953, -4193.72363, 286.17593),
(241327,6,1816.68005, -4173.06005, 294.94101);

DELETE FROM conditions WHERE SourceTypeOrReferenceId = 13 AND SourceEntry = 43109;
INSERT INTO `conditions` (`SourceTypeOrReferenceId`, `SourceGroup`, `SourceEntry`, `ElseGroup`, `ConditionTypeOrReference`, `ConditionValue1`, `ConditionValue2`, `ConditionValue3`, `ErrorTextId`, `ScriptName`, `Comment`, `NegativeCondition`) VALUES
(13, 1, 43109, 0, 31, 3, 24110, 0, 0, '', "Throw Torch",0),
(13, 1, 43109, 0, 31, 3, 24110, 100295, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100311, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100313, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100294, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100289, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100286, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100306, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100287, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100303, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100271, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100283, 0, '', "Throw Torch",1),
(13, 1, 43109, 0, 31, 3, 24110, 100305, 0, '', "Throw Torch",1);

DELETE FROM `conditions` WHERE `SourceEntry` IN (43068) AND `SourceTypeOrReferenceId`=13;
INSERT INTO `conditions` (`SourceTypeOrReferenceId`, `SourceGroup`, `SourceEntry`, `SourceId`, `ElseGroup`, `ConditionTypeOrReference`, `ConditionTarget`, `ConditionValue1`, `ConditionValue2`, `ConditionValue3`, `NegativeCondition`, `ErrorType`, `ErrorTextId`, `ScriptName`, `Comment`) VALUES
(13, 1, 43068, 0, 0, 31, 0, 3, 24087, 0, 0, 0, 0, "", "Spell Brave's Flare effect"),
(13, 1, 43068, 0, 1, 31, 0, 3, 24092, 0, 0, 0, 0, "", "Spell Brave's Flare effect"),
(13, 1, 43068, 0, 2, 31, 0, 3, 24093, 0, 0, 0, 0, "", "Spell Brave's Flare effect"),
(13, 1, 43068, 0, 3, 31, 0, 3, 24094, 0, 0, 0, 0, "", "Spell Brave's Flare effect");

DELETE FROM `creature_template_movement` WHERE `CreatureId`= 24132;
INSERT INTO `creature_template_movement` (`CreatureId`,`Ground`,`Swim`,`Flight`,`Rooted`) VALUES
(24132, 0, 0, 1, 0);

UPDATE `smart_scripts`  SET `target_param2`=35 WHERE `entryorguid` IN (24100,24098,24102) AND `source_type`=0 AND `id` IN (1,2);