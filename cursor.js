import React from './lib/dom/preactComponent.js';
//import cursorPng from '../assets/cursor.png'
const cursorPng = '/static/ruler/cursor.png';

const Cursor = ({ horizontal }) =>
  horizontal
    ? h('img',
        {
          src: 'static/ruler/cursor.svg',
          alt: 'cursor',
          height: '100%',
          width: '30',
          style: { objectFit: 'cover'  }
        } /*, h(CursorImage)*/
      )
    : h('img',
        {
          src: 'static/ruler/cursor.svg',
          alt: 'cursor',
          width: '512',
          height: '512',
          style: { height: '30px', width: 'auto', objectFit: 'cover', transform: 'rotate(90deg)'  }
        } /*,  h(CursorImage)*/
      );

function CursorImage(props) {
  return h('svg',
    {
      height: '512',
      width: '512',
      viewBox: '0 0 135.47 135.47'
      //xmlns: "http://www.w3.org/2000/svg"
    },
    h('path', {
      strokeWidth: '.265',stroke: '#f00', 
      d: 'M46.915 135.095c-1.388-.635-1.67-1.538-1.67-5.325 0-4.824.416-5.404 3.889-5.414 6.182-.017 9.965-2.624 12.224-8.425l.688-1.767.15-40.878h-4.52c-6.534 0-6.875-.276-6.875-5.556s.341-5.557 6.875-5.557h4.52l-.15-40.878-.688-1.767c-2.259-5.801-6.041-8.408-12.224-8.425-3.482-.01-3.89-.584-3.889-5.467.002-5.27.629-5.86 5.881-5.527 6.343.402 11.335 2.672 15.351 6.983l1.257 1.348 1.257-1.348c2.8-3.007 6.643-5.311 10.5-6.297 1.12-.287 3.426-.592 5.127-.68 2.678-.136 3.215-.093 4.014.325 1.387.725 1.583 1.367 1.587 5.196.007 4.883-.402 5.458-3.885 5.467-3.857.01-6.226.807-8.582 2.885-1.496 1.32-2.728 3.194-3.642 5.54l-.688 1.767-.15 40.878h4.52c6.534 0 6.875.276 6.875 5.557 0 5.28-.341 5.556-6.875 5.556h-4.52l.15 40.878.688 1.767c.914 2.347 2.146 4.22 3.642 5.54 2.356 2.078 4.725 2.875 8.582 2.885 3.481.01 3.89.584 3.888 5.467-.002 5.27-.628 5.86-5.88 5.527-6.343-.401-11.336-2.672-15.352-6.982l-1.256-1.35-1.257 1.35c-2.801 3.006-6.644 5.31-10.5 6.297-2.764.706-7.923.951-9.062.43z'
    })
  );
}

export default Cursor;
