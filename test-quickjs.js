import { Rect } from './build/x86_64-linux-gnu/quickjs-rect.so';
import { Point } from './build/x86_64-linux-gnu/quickjs-point.so';
import { Size } from './build/x86_64-linux-gnu/quickjs-size.so';
import { Mat } from './build/x86_64-linux-gnu/quickjs-mat.so';
import { PointIterator } from './build/x86_64-linux-gnu/quickjs-point-iterator.so';
import { inspect } from './inspect.js';
//import { Contour } from "./build/x86_64-linux-gnu/quickjs-contour.so";

console.log('test:', inspect({ Point, Size, Rect, Mat }));
