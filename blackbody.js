import { RGBA, HSLA } from './lib/color.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';

const blackbody_color = new Map([
  [1000, [1, 0.18172716, 0]],
  [1100, [1, 0.25503671, 0]],
  [1200, [1, 0.30942099, 0]],
  [1300, [1, 0.35357379, 0]],
  [1400, [1, 0.39091524, 0]],
  [1500, [1, 0.42322816, 0]],
  [1600, [1, 0.45159884, 0]],
  [1700, [1, 0.47675916, 0]],
  [1800, [1, 0.49923747, 0]],
  [1900, [1, 0.51943421, 0]],
  [2000, [1, 0.54360078, 0.08679949]],
  [2100, [1, 0.56618736, 0.14065513]],
  [2200, [1, 0.58734976, 0.18362641]],
  [2300, [1, 0.60724493, 0.22137978]],
  [2400, [1, 0.62600248, 0.2559195]],
  [2500, [1, 0.64373109, 0.28819679]],
  [2600, [1, 0.66052319, 0.31873863]],
  [2700, [1, 0.67645822, 0.34786758]],
  [2800, [1, 0.69160518, 0.37579588]],
  [2900, [1, 0.70602449, 0.40267128]],
  [3000, [1, 0.71976951, 0.42860152]],
  [3100, [1, 0.7328876, 0.45366838]],
  [3200, [1, 0.74542112, 0.47793608]],
  [3300, [1, 0.75740814, 0.50145662]],
  [3400, [1, 0.76888303, 0.52427322]],
  [3500, [1, 0.77987699, 0.54642268]],
  [3600, [1, 0.79041843, 0.56793692]],
  [3700, [1, 0.80053332, 0.58884417]],
  [3800, [1, 0.81024551, 0.60916971]],
  [3900, [1, 0.81957693, 0.62893653]],
  [4000, [1, 0.82854786, 0.6481657]],
  [4100, [1, 0.83717703, 0.66687674]],
  [4200, [1, 0.84548188, 0.68508786]],
  [4300, [1, 0.85347859, 0.70281616]],
  [4400, [1, 0.86118227, 0.72007777]],
  [4500, [1, 0.86860704, 0.73688797]],
  [4600, [1, 0.87576611, 0.75326132]],
  [4700, [1, 0.88267187, 0.76921169]],
  [4800, [1, 0.88933596, 0.78475236]],
  [4900, [1, 0.89576933, 0.79989606]],
  [5000, [1, 0.9019823, 0.81465502]],
  [5100, [1, 0.90963069, 0.8283821]],
  [5200, [1, 0.91710889, 0.84190889]],
  [5300, [1, 0.92441842, 0.85523742]],
  [5400, [1, 0.93156127, 0.86836903]],
  [5500, [1, 0.93853986, 0.88130458]],
  [5600, [1, 0.94535695, 0.8940447]],
  [5700, [1, 0.95201559, 0.90658983]],
  [5800, [1, 0.95851906, 0.91894041]],
  [5900, [1, 0.96487079, 0.9310969]],
  [6000, [1, 0.97107439, 0.94305985]],
  [6100, [1, 0.97713351, 0.95482993]],
  [6200, [1, 0.98305189, 0.96640795]],
  [6300, [1, 0.98883326, 0.97779486]],
  [6400, [1, 0.99448139, 0.98899179]],
  [6500, [1, 1, 1]],
  [6600, [0.98947904, 0.99348723, 1]],
  [6700, [0.97940448, 0.98722715, 1]],
  [6800, [0.96975025, 0.98120637, 1]],
  [6900, [0.96049223, 0.9754124, 1]],
  [7000, [0.95160805, 0.96983355, 1]],
  [7100, [0.94303638, 0.96443333, 1]],
  [7200, [0.93480451, 0.9592308, 1]],
  [7300, [0.92689056, 0.95421394, 1]],
  [7400, [0.91927697, 0.9493733, 1]],
  [7500, [0.91194747, 0.94470005, 1]],
  [7600, [0.9048869, 0.94018594, 1]],
  [7700, [0.89808115, 0.93582323, 1]],
  [7800, [0.8915171, 0.93160469, 1]],
  [7900, [0.88518247, 0.92752354, 1]],
  [8000, [0.87906581, 0.9235734, 1]],
  [8100, [0.8731564, 0.91974827, 1]],
  [8200, [0.86744421, 0.91604254, 1]],
  [8300, [0.86191983, 0.91245088, 1]],
  [8400, [0.85657444, 0.90896831, 1]],
  [8500, [0.85139976, 0.90559011, 1]],
  [8600, [0.84638799, 0.90231183, 1]],
  [8700, [0.8415318, 0.89912926, 1]],
  [8800, [0.8368243, 0.89603843, 1]],
  [8900, [0.83225897, 0.89303558, 1]],
  [9000, [0.82782969, 0.89011714, 1]],
  [9100, [0.82353066, 0.88727974, 1]],
  [9200, [0.81935641, 0.88452017, 1]],
  [9300, [0.81530175, 0.88183541, 1]],
  [9400, [0.8113618, 0.87922257, 1]],
  [9500, [0.80753191, 0.87667891, 1]],
  [9600, [0.80380769, 0.87420182, 1]],
  [9700, [0.80018497, 0.87178882, 1]],
  [9800, [0.7966598, 0.86943756, 1]],
  [9900, [0.79322843, 0.86714579, 1]],
  [10000, [0.78988728, 0.86491137, 1]],
  [10100, [0.78663296, 0.86273225, 1]],
  [10200, [0.78346225, 0.8606065, 1]],
  [10300, [0.78037207, 0.85853224, 1]],
  [10400, [0.7773595, 0.85650771, 1]],
  [10500, [0.77442176, 0.85453121, 1]],
  [10600, [0.77155617, 0.85260112, 1]],
  [10700, [0.76876022, 0.85071588, 1]],
  [10800, [0.76603147, 0.84887402, 1]],
  [10900, [0.76336762, 0.84707411, 1]],
  [11000, [0.76076645, 0.84531479, 1]],
  [11100, [0.75822586, 0.84359476, 1]],
  [11200, [0.75574383, 0.84191277, 1]],
  [11300, [0.75331843, 0.84026762, 1]],
  [11400, [0.7509478, 0.83865816, 1]],
  [11500, [0.74863017, 0.83708329, 1]],
  [11600, [0.74636386, 0.83554194, 1]],
  [11700, [0.74414722, 0.83403311, 1]],
  [11800, [0.74197871, 0.83255582, 1]],
  [11900, [0.73985682, 0.83110912, 1]],
  [12000, [0.73778012, 0.82969211, 1]],
  [12100, [0.73574723, 0.82830393, 1]],
  [12200, [0.73375683, 0.82694373, 1]],
  [12300, [0.73180765, 0.82561071, 1]],
  [12400, [0.72989845, 0.8243041, 1]],
  [12500, [0.72802807, 0.82302316, 1]],
  [12600, [0.72619537, 0.82176715, 1]],
  [12700, [0.72439927, 0.82053539, 1]],
  [12800, [0.72263872, 0.81932722, 1]],
  [12900, [0.7209127, 0.81814197, 1]],
  [13000, [0.71922025, 0.81697905, 1]],
  [13100, [0.71756043, 0.81583783, 1]],
  [13200, [0.71593234, 0.81471775, 1]],
  [13300, [0.7143351, 0.81361825, 1]],
  [13400, [0.71276788, 0.81253878, 1]],
  [13500, [0.71122987, 0.81147883, 1]],
  [13600, [0.70972029, 0.81043789, 1]],
  [13700, [0.70823838, 0.80941546, 1]],
  [13800, [0.70678342, 0.80841109, 1]],
  [13900, [0.70535469, 0.80742432, 1]],
  [14000, [0.70395153, 0.80645469, 1]],
  [14100, [0.70257327, 0.8055018, 1]],
  [14200, [0.70121928, 0.80456522, 1]],
  [14300, [0.69988894, 0.80364455, 1]],
  [14400, [0.69858167, 0.80273941, 1]],
  [14500, [0.69729688, 0.80184943, 1]],
  [14600, [0.69603402, 0.80097423, 1]],
  [14700, [0.69479255, 0.80011347, 1]],
  [14800, [0.69357196, 0.79926681, 1]],
  [14900, [0.69237173, 0.79843391, 1]],
  [15000, [0.69119138, 0.79761446, 1]],
  [15100, [0.69003044, 0.79680814, 1]],
  [15200, [0.68888844, 0.79601466, 1]],
  [15300, [0.68776494, 0.79523371, 1]],
  [15400, [0.68665951, 0.79446502, 1]],
  [15500, [0.68557173, 0.7937083, 1]],
  [15600, [0.68450119, 0.7929633, 1]],
  [15700, [0.68344751, 0.79222975, 1]],
  [15800, [0.68241029, 0.7915074, 1]],
  [15900, [0.68138918, 0.790796, 1]],
  [16000, [0.6803838, 0.79009531, 1]],
  [16100, [0.67939381, 0.78940511, 1]],
  [16200, [0.67841888, 0.78872517, 1]],
  [16300, [0.67745866, 0.78805526, 1]],
  [16400, [0.67651284, 0.78739518, 1]],
  [16500, [0.67558112, 0.78674472, 1]],
  [16600, [0.67466317, 0.78610368, 1]],
  [16700, [0.67375872, 0.78547186, 1]],
  [16800, [0.67286748, 0.78484907, 1]],
  [16900, [0.67198916, 0.78423512, 1]],
  [17000, [0.6711235, 0.78362984, 1]],
  [17100, [0.67027024, 0.78303305, 1]],
  [17200, [0.66942911, 0.78244457, 1]],
  [17300, [0.66859988, 0.78186425, 1]],
  [17400, [0.66778228, 0.78129191, 1]],
  [17500, [0.6669761, 0.7807274, 1]],
  [17600, [0.6661811, 0.78017057, 1]],
  [17700, [0.66539706, 0.77962127, 1]],
  [17800, [0.66462376, 0.77907934, 1]],
  [17900, [0.66386098, 0.77854465, 1]],
  [18000, [0.66310852, 0.77801705, 1]],
  [18100, [0.66236618, 0.77749642, 1]],
  [18200, [0.66163375, 0.77698261, 1]],
  [18300, [0.66091106, 0.77647551, 1]],
  [18400, [0.66019791, 0.77597498, 1]],
  [18500, [0.65949412, 0.7754809, 1]],
  [18600, [0.65879952, 0.77499315, 1]],
  [18700, [0.65811392, 0.77451161, 1]],
  [18800, [0.65743716, 0.77403618, 1]],
  [18900, [0.65676908, 0.77356673, 1]],
  [19000, [0.65610952, 0.77310316, 1]],
  [19100, [0.65545831, 0.77264537, 1]],
  [19200, [0.6548153, 0.77219324, 1]],
  [19300, [0.65418036, 0.77174669, 1]],
  [19400, [0.65355332, 0.7713056, 1]],
  [19500, [0.65293404, 0.77086988, 1]],
  [19600, [0.6523224, 0.77043944, 1]],
  [19700, [0.65171824, 0.77001419, 1]],
  [19800, [0.65112144, 0.76959404, 1]],
  [19900, [0.65053187, 0.76917889, 1]],
  [20000, [0.64994941, 0.76876866, 1]],
  [20100, [0.64937392, 0.76836326, 1]],
  [20200, [0.64880528, 0.76796263, 1]],
  [20300, [0.64824339, 0.76756666, 1]],
  [20400, [0.64768812, 0.76717529, 1]],
  [20500, [0.64713935, 0.76678844, 1]],
  [20600, [0.64659699, 0.76640603, 1]],
  [20700, [0.64606092, 0.76602798, 1]],
  [20800, [0.64553103, 0.76565424, 1]],
  [20900, [0.64500722, 0.76528472, 1]],
  [21000, [0.64448939, 0.76491935, 1]],
  [21100, [0.64397745, 0.76455808, 1]],
  [21200, [0.64347129, 0.76420082, 1]],
  [21300, [0.64297081, 0.76384753, 1]],
  [21400, [0.64247594, 0.76349813, 1]],
  [21500, [0.64198657, 0.76315256, 1]],
  [21600, [0.64150261, 0.76281076, 1]],
  [21700, [0.64102399, 0.76247267, 1]],
  [21800, [0.64055061, 0.76213824, 1]],
  [21900, [0.64008239, 0.7618074, 1]],
  [22000, [0.63961926, 0.7614801, 1]],
  [22100, [0.63916112, 0.76115628, 1]],
  [22200, [0.6387079, 0.7608359, 1]],
  [22300, [0.63825953, 0.7605189, 1]],
  [22400, [0.63781592, 0.76020522, 1]],
  [22500, [0.63737701, 0.75989482, 1]],
  [22600, [0.63694273, 0.75958764, 1]],
  [22700, [0.63651299, 0.75928365, 1]],
  [22800, [0.63608774, 0.75898278, 1]],
  [22900, [0.63566691, 0.75868499, 1]],
  [23000, [0.63525042, 0.75839025, 1]],
  [23100, [0.63483822, 0.75809849, 1]],
  [23200, [0.63443023, 0.75780969, 1]],
  [23300, [0.63402641, 0.75752379, 1]],
  [23400, [0.63362667, 0.75724075, 1]],
  [23500, [0.63323097, 0.75696053, 1]],
  [23600, [0.63283925, 0.7566831, 1]],
  [23700, [0.63245144, 0.7564084, 1]],
  [23800, [0.63206749, 0.75613641, 1]],
  [23900, [0.63168735, 0.75586707, 1]],
  [24000, [0.63131096, 0.75560036, 1]],
  [24100, [0.63093826, 0.75533624, 1]],
  [24200, [0.6305692, 0.75507467, 1]],
  [24300, [0.63020374, 0.75481562, 1]],
  [24400, [0.62984181, 0.75455904, 1]],
  [24500, [0.62948337, 0.75430491, 1]],
  [24600, [0.62912838, 0.75405319, 1]],
  [24700, [0.62877678, 0.75380385, 1]],
  [24800, [0.62842852, 0.75355685, 1]],
  [24900, [0.62808356, 0.75331217, 1]],
  [25000, [0.62774186, 0.75306977, 1]],
  [25100, [0.62740336, 0.75282962, 1]]
]);

async function main(arg = '3400k') {
  await ConsoleSetup({ colors: true, depth: Infinity });

  const colorTemp = Util.roundTo(parseInt(arg), 100);

  const eq2_gamma = 1.0,
    contrast = 1.0,
    brightness = 0,
    saturation = 1.0,
    weight = 1.0;

  const gamma = blackbody_color.get(colorTemp);
  const [r = 1.0, g = 1.0, b = 1.0] = gamma;

  console.log('Gamma RGB:', { r, g, b });
  console.log(
    `-vf eq2=${[eq2_gamma, contrast, brightness, saturation, r, g, b, weight]
      .map(n => Util.roundTo(n, 0.0001, 5))
      .join(':')}`
  );
}

Util.callMain(main, true);
