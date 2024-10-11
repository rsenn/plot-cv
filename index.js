{
  tagName: '?xml',
  attributes: {
    version: '1.0',
    encoding: 'utf-8'
  },
  children: [
    {
      tagName: '!DOCTYPE html'
    },
    {
      tagName: 'html',
      attributes: {},
      children: [
        {
          tagName: 'head',
          attributes: {},
          children: [
            {
              tagName: 'title',
              attributes: {},
              children: [
                'webakeit'
              ]
            },
            {
              tagName: 'link',
              attributes: {
                href: 'static/fonts/LibreCaslonText-Regular.css',
                rel: 'stylesheet'
              }
            },
            {
              tagName: 'link',
              attributes: {
                href: 'static/css/webakeit.css',
                rel: 'stylesheet'
              }
            },
            {
              tagName: 'style',
              attributes: {
                type: 'text/css'
              },
              children: []
            },
            {
              tagName: 'script',
              attributes: {
                type: 'module',
                src: './webakeit.js'
              },
              children: []
            }
          ]
        },
        {
          tagName: 'body',
          attributes: {},
          children: [
            {
              tagName: 'div',
              attributes: {
                'data-name': 'home',
                class: 'home'
              },
              children: [
                {
                  tagName: 'div',
                  attributes: {
                    id: 'header'
                  },
                  children: [
                    {
                      tagName: 'div',
                      attributes: {
                        'data-name': 'home'
                      },
                      children: [
                        'Menu'
                      ]
                    },
                    {
                      tagName: 'div',
                      attributes: {
                        'data-name': 'blog'
                      },
                      children: [
                        'Blog',
                        {
                          tagName: 'br',
                          attributes: {}
                        },
                        'News'
                      ]
                    },
                    {
                      tagName: 'div',
                      attributes: {
                        'data-name': 'haus'
                      },
                      children: [
                        'Das Haus'
                      ]
                    },
                    {
                      tagName: 'div',
                      attributes: {
                        'data-name': 'spenden'
                      },
                      children: [
                        'Werde',
                        {
                          tagName: 'br',
                          attributes: {}
                        },
                        'Spender'
                      ]
                    }
                  ]
                },
                {
                  tagName: 'div',
                  attributes: {},
                  children: [
                    '&nbsp;'
                  ]
                }
              ]
            },
            {
              tagName: 'div',
              attributes: {
                'data-name': 'blog',
                class: 'blog'
              },
              children: [
                {
                  tagName: 'img',
                  attributes: {
                    class: 'close',
                    src: 'static/img/schliessen.svg',
                    border: '0',
                    onclick: 'PopPage();'
                  }
                }
              ]
            }
          ]
        }
      ]
    }
  ]
}