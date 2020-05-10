import { html, render, Component, useState, useCallback } from "../modules/htm/preact/standalone.mjs";

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
        <${itemComponent} key=${key || i} className=${classNames(itemClass || className + "-item", (name + "").replace(/.*\./, ""))} active=${i == active} onPush=${pushHandler(i)} text="${name}" ...${item} />
      `
  );
  return html`<${Container} className=${classNames("panel", className)} ...${props}>${children}</${Container}>`;
};

export const Button = ({ caption, fn }) => html`
  <${Overlay} className="button" text=${caption} onPush=${state => (state ? fn(state) : undefined)} />
`;

export const Label = ({ className = "label", caption, children, ...props }) =>
  html`
    <div className=${className} ...${props}>${caption}${children}</div>
  `;

export const Item = ({ className = "item", label, children, ...props }) => html`
            <${Overlay} className=${className} text=${label} ...${props}><${Label} caption=${label} />
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

export const File = ({ name, i, key, signal, data, ...props }) => {
  const [loaded, setLoaded] = useState(NaN);
  if(signal) signal.subscribe(data => setLoaded(data.percent));
  const pushHandler = async state => {
    console.log(`loading "${name}"...`);
    await load(name);
  };
  let id = name || key || i;
  if(id) id = (id + "").replace(/[^._A-Za-z0-9]/g, "-");
  return html`
              <${Item} className=file id=${id} data-filename="${name}" onPush=${pushHandler} ...${props}>${name}<${Progress} className=${!isNaN(loaded) ? "visible" : "hidden"} percent=${loaded} /></${Item}>
            `;
};

export const FileList = ({ files, onChange, onActive, ...props }) => {
  const [active, setActive] = useState(true);

  onActive.subscribe(value => setActive(value));

  return html`
    <div className=${classNames("sidebar", active ? "active" : "inactive")} style=${{ transform: active ? "none" : "translateX(-100%)" }}>
      <${Chooser}
        className="list"
        itemComponent=${File}
        itemClass="file hcenter"
        items=${files}
        onChange=${(...args) => {
          onActive(false);
          onChange(...args);
        }}
        ...${props}
      />
    </div>
  `;
};
