import { h, html, render, Component, useState, useCallback } from "../modules/htm/preact/standalone.mjs";

export const Overlay = ({ className = "overlay", active = false, onPush, text, children, ...props }) => {
  const [pushed, setPushed] = useState(false);
  const events = MouseEvents(
    MouseHandler((e, state) => {
      const prev = pushed;
      if(!e.type.endsWith("down") && !e.type.endsWith("up")) return;
      setPushed(state);
      //  console.log(`overlay pushed=${pushed} active=${active}:`, e.target);
      return typeof onPush == "function" ? onPush(e, state) : null;
    })
  );

  return html`
    <div className=${classNames(className, pushed && "pushed", active ? "active" : "inactive")} ...${props} ...${events}>
      ${text} ${children}
    </div>
  `;
};

export const Container = ({ className = "panel", children, ...props }) => {
  useCallback(() => console.log("re-render panel"));
  return html`
    <div className=${className} ...${props}>${children}</div>
  `;
};

export const Chooser = ({ className = "list", itemClass = "item", itemComponent = Overlay, items, onChange = () => {}, onPush = () => {}, ...props }) => {
  const [active, setActive] = useState(-1);
  const pushHandler = i => (e, state) => {
    const prev = active;
    state == true && setActive(i);
    if(i != prev && e.type.endsWith("down")) onChange(e, items[i], i);
    onPush(e, i, state);
  };
  const bar = html``;
  const children = items.map(
    ({ name, i, data, signal, ...item }, key) =>
      html`
        <${itemComponent} key=${key || i} className=${classNames(itemClass || className + "-item", (name + "").replace(/.*\./, ""))} active=${i == active} onPush=${pushHandler(i)} label="${name}" ...${item} />
      `
  );
  return html`<${Container} className=${classNames("panel", className)} ...${props}>${children}</${Container}>`;
};

export const Button = ({ caption, fn }) => html`
  <${Overlay} className="button" text=${caption} onPush=${state => (state ? fn(state) : undefined)} />
`;

export const Label = ({ className = "label", text, children, ...props }) =>
  html`
    <div className=${className} ...${props}>${text}${children}</div>
  `;

export const Item = ({ className = "item", label, icon, children, ...props }) => html`
            <${Overlay} className=${className}  ...${props}>
                <${Label} text=${icon} ...${props}>${label}</${Label}>
            </${Overlay}>
          `;

export const Icon = ({ className = "icon", caption, image, ...props }) => html`
  <div className=${className} ...${props}>${caption}<img src=${image} /></div>
`;

export const Progress = ({ className, percent, ...props }) => html`
                <${Overlay} className=${classNames("progress", "center", className)} text=${percent + "%"} style=${{
  position: "relative",
  width: "100%",
  height: "1.5em",
  border: "1px solid black",
  textAlign: "center",
  zIndex: "99"
}}><div className=${classNames("progress-bar", "fill")} style=${{
  width: percent + "%",
  position: "absolute",
  left: "0px",
  top: "0px",
  zIndex: "98"
}}></div></${Overlay}>`;

const SchematicIcon = props => html`
  <svg viewBox="0 0 40 40" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <clipPath id="Sch_svg__a">
        <path d="M0 32h32V0H0v32z" />
      </clipPath>
    </defs>
    <g clipPath="url(#Sch_svg__a)" transform="matrix(1.25 0 0 -1.25 0 40)">
      <path
        d="M26.463 27c0 .817-.308 1.555-.847 2.125a2.785 2.785 0 01-2.038.873H10.212c-.774 0-1.494-.3-2.03-.873A3.028 3.028 0 017.34 27V4.996c0-1.648 1.29-2.994 2.873-2.994H20.727v6.143h5.736V27zm1.904-19.07a2.182 2.182 0 00-.56-1.34L22.075.586A1.9 1.9 0 0020.71 0H10.212C7.557 0 5.43 2.241 5.43 4.996V27c0 1.388.54 2.642 1.403 3.547.87.892 2.053 1.453 3.379 1.453h13.366a4.688 4.688 0 003.382-1.453A5.137 5.137 0 0028.367 27V7.93z"
        fill="#444443"
      />
      <path d="M26.459 12.143H3.628v14h22.831v-14z" fill="#b4b3b3" />
      <path
        d="M10.956 16.95c.23-.297.401-.561.401-.952 0-.86-.654-1.297-1.445-1.297H7.387c-.437 0-.815.08-1.16.368-.31.252-.539.665-.539 1.067 0 .367.206.689.608.689.333 0 .54-.275.54-.585 0-.345.183-.391.505-.391h2.49c.092 0 .378-.035.378.139 0 .09-.16.263-.206.332l-3.937 5.06c-.24.31-.378.54-.378.943 0 .459.218.849.608 1.1.321.208.585.208.952.208h2.434c.436 0 .803-.092 1.136-.38.3-.263.54-.688.54-1.09 0-.367-.184-.665-.597-.665-.345 0-.552.218-.552.539 0 .39-.194.448-.55.448H7.226c-.126 0-.39.034-.39-.16 0-.104.126-.242.171-.311l3.95-5.061zM14.902 14.701c-.861 0-1.354.344-1.722 1.091l-1.055 2.134c-.196.39-.31.757-.31 1.205 0 .505.09.815.32 1.274l1.057 2.124c.39.791.884 1.101 1.767 1.101h1.95c.253 0 .576-.137.576-.574 0-.413-.311-.574-.575-.574h-1.916c-.494 0-.574-.046-.792-.482l-1-1.997c-.148-.309-.24-.516-.24-.872 0-.322.104-.516.24-.792l.976-1.985c.23-.47.346-.506.86-.506h1.872c.252 0 .575-.137.575-.573 0-.413-.311-.574-.575-.574h-2.008zM19.838 18.592v-3.317c0-.368-.16-.574-.573-.574-.413 0-.574.206-.574.574v7.782c0 .367.16.574.574.574.413 0 .573-.207.573-.574V19.74h3.375v3.317c0 .367.16.574.573.574.413 0 .574-.207.574-.574v-7.782c0-.368-.161-.574-.574-.574-.413 0-.573.206-.573.574v3.317h-3.375z"
        fill="#bf272f"
      />
    </g>
  </svg>
`;

export const BoardIcon = props => html`
  <svg viewBox="0 0 40 40" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <clipPath id="Brd_svg__a">
        <path d="M0 32h32V0H0v32z" />
      </clipPath>
    </defs>
    <g clipPath="url(#Brd_svg__a)" transform="matrix(1.25 0 0 -1.25 0 40)">
      <path
        d="M26.463 27c0 .817-.308 1.555-.847 2.125a2.785 2.785 0 01-2.038.873H10.212c-.774 0-1.494-.3-2.03-.873A3.028 3.028 0 017.34 27V4.996c0-1.648 1.29-2.994 2.873-2.994H20.727v6.143h5.736V27zm1.904-19.07a2.182 2.182 0 00-.56-1.34L22.075.586A1.9 1.9 0 0020.71 0H10.212C7.557 0 5.43 2.241 5.43 4.996V27c0 1.388.54 2.642 1.403 3.547.87.892 2.053 1.453 3.379 1.453h13.366a4.688 4.688 0 003.382-1.453A5.137 5.137 0 0028.367 27V7.93z"
        fill="#444443"
      />
      <path d="M26.463 12.164H3.633v14h22.83v-14z" fill="#b4b3b3" />
      <path
        d="M6.99 15.835h2.087c.835 0 1.322.37 1.322 1.252 0 .974-.266 1.52-1.322 1.52H6.99v-2.773zm0 3.931h2.087c.835 0 1.322.372 1.322 1.253 0 .974-.266 1.519-1.322 1.519H6.99v-2.772zm-1.16 3.932h3.247c1.705 0 2.482-1.044 2.482-2.68 0-.671-.15-1.286-.765-1.831.66-.626.765-1.241.765-2.1 0-1.52-1.009-2.411-2.482-2.411H5.83v9.022zM13.95 20.729h2.574c.325 0 .836.023.836.487v.8c0 .464-.476.522-.824.522H13.95v-1.809zm0-5.474c0-.37-.163-.579-.58-.579-.418 0-.58.209-.58.579v8.443h3.897c.917 0 1.833-.65 1.833-1.682v-.8c0-1.091-.904-1.647-1.833-1.647H15.62l2.609-3.735c.127-.173.29-.393.29-.614 0-.324-.267-.544-.591-.544-.232 0-.383.138-.51.324l-3.201 4.569h-.267v-4.314zM21.654 15.835h.221c.488 0 .603.058.812.486l1.02 2.065c.13.267.22.465.22.755 0 .394-.127.614-.289.95l-.963 1.96c-.244.499-.347.487-.88.487h-.14v-6.703zm-1.158 6.702h-.43c-.383 0-.707.14-.707.58 0 .418.313.58.672.58h1.937c.905 0 1.356-.37 1.75-1.16l1.022-2.04c.23-.476.348-.812.348-1.346 0-.463-.127-.858-.336-1.275l-1.057-2.11c-.404-.825-.928-1.09-1.832-1.09H19.94c-.303 0-.58.161-.58.58 0 .416.277.578.58.578h.557v6.704z"
        fill="#42ab3d"
      />
    </g>
  </svg>
`;

export const File = ({ name, i, key, signal, data, ...props }) => {
  const [loaded, setLoaded] = useState(NaN);
  if(signal) signal.subscribe(data => setLoaded(data.percent));
  const pushHandler = async state => {
    console.log(`loading "${name}"...`);
    await load(name);
  };
  let id = name || key || i;
  let style = { width: "20px", height: "20px" };
  let icon = /brd$/i.test(props.className) ? h(BoardIcon, { style }) : h(SchematicIcon, { style });
  icon = h("div", { style }, icon);
  if(id) id = (id + "").replace(/[^._A-Za-z0-9]/g, "-");
  //data = signal();
  //console.log(`File`, { id, data, ...props });

  return html`
              <${Item} className=file id=${id} data-filename="${name}" onPush=${pushHandler} label=${name} icon=${icon} ...${props}>
              <${Progress} className=${!isNaN(loaded) ? "visible" : "hidden"} percent=${loaded} />
              </${Item}>
            `;
};

export const FileList = ({ files, onChange, onActive, ...props }) => {
  const [active, setActive] = useState(true);
  const [items, setItems] = useState(files());

  files.subscribe(value => setItems(value));

  onActive.subscribe(value => setActive(value));

  return html`
    <div className=${classNames("sidebar", active ? "active" : "inactive")}>
      <${Chooser}
        className="list"
        itemComponent=${File}
        itemClass="file hcenter"
        items=${items}
        onChange=${(...args) => {
          onActive(false);
          onChange(...args);
        }}
        ...${props}
      />
    </div>
  `;
};

export default { Overlay, Container, Chooser, Button, Label, Item, Icon, Progress, BoardIcon, File, FileList };
