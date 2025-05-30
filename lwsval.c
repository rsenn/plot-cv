#include <stdio.h>
#include <stddef.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/un.h>
#include <netpacket/packet.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <setjmp.h>
#include <glob.h>
#include <signal.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdint.h>
#include <limits.h>
#include <stdlib.h>
#include <libwebsockets.h>
#include <errno.h>
#include <sys/ptrace.h>
#include <sys/inotify.h>
#include <portmidi.h>
#include <linux/random.h>
#include <zlib.h>
#include <dns.h>

static inline int
escape_char_pred(int c) {
  switch(c) {
    case 8: return 'b';
    case 9: return 't';
    case 10: return 'n';
    case 11: return 'v';
    case 12: return 'f';
    case 13: return 'r';
    case 39: return '\'';
    case 92: return '\\';
  }
  if(c < 0x20 || c == 127)
    return 'x';

  return 0;
}

typedef struct numbers_s {
  union {
    uint32_t fl_i;
    float fl;
  };
  short sh;
  union {
    uint64_t db_i;
    double db;
  };
} numbers_t;

int
main() {
  printf("WSI_TOKEN_GET_URI = %i\n", WSI_TOKEN_GET_URI);
  printf("WSI_TOKEN_POST_URI = %i\n", WSI_TOKEN_POST_URI);
  printf("WSI_TOKEN_GET_URI = %i\n", WSI_TOKEN_GET_URI);
  printf("WSI_TOKEN_POST_URI = %i\n", WSI_TOKEN_POST_URI);
  printf("WSI_TOKEN_OPTIONS_URI = %i\n", WSI_TOKEN_OPTIONS_URI);
  printf("WSI_TOKEN_HOST = %i\n", WSI_TOKEN_HOST);
  printf("WSI_TOKEN_CONNECTION = %i\n", WSI_TOKEN_CONNECTION);
  printf("WSI_TOKEN_UPGRADE = %i\n", WSI_TOKEN_UPGRADE);
  printf("WSI_TOKEN_ORIGIN = %i\n", WSI_TOKEN_ORIGIN);
  printf("WSI_TOKEN_DRAFT = %i\n", WSI_TOKEN_DRAFT);
  printf("WSI_TOKEN_CHALLENGE = %i\n", WSI_TOKEN_CHALLENGE);
  printf("WSI_TOKEN_EXTENSIONS = %i\n", WSI_TOKEN_EXTENSIONS);
  printf("WSI_TOKEN_KEY1 = %i\n", WSI_TOKEN_KEY1);
  printf("WSI_TOKEN_KEY2 = %i\n", WSI_TOKEN_KEY2);
  printf("WSI_TOKEN_PROTOCOL = %i\n", WSI_TOKEN_PROTOCOL);
  printf("WSI_TOKEN_ACCEPT = %i\n", WSI_TOKEN_ACCEPT);
  printf("WSI_TOKEN_NONCE = %i\n", WSI_TOKEN_NONCE);
  printf("WSI_TOKEN_HTTP = %i\n", WSI_TOKEN_HTTP);
  printf("WSI_TOKEN_HTTP2_SETTINGS = %i\n", WSI_TOKEN_HTTP2_SETTINGS);
  printf("WSI_TOKEN_HTTP_ACCEPT = %i\n", WSI_TOKEN_HTTP_ACCEPT);
  printf("WSI_TOKEN_HTTP_AC_REQUEST_HEADERS = %i\n", WSI_TOKEN_HTTP_AC_REQUEST_HEADERS);
  printf("WSI_TOKEN_HTTP_IF_MODIFIED_SINCE = %i\n", WSI_TOKEN_HTTP_IF_MODIFIED_SINCE);
  printf("WSI_TOKEN_HTTP_IF_NONE_MATCH = %i\n", WSI_TOKEN_HTTP_IF_NONE_MATCH);
  printf("WSI_TOKEN_HTTP_ACCEPT_ENCODING = %i\n", WSI_TOKEN_HTTP_ACCEPT_ENCODING);
  printf("WSI_TOKEN_HTTP_ACCEPT_LANGUAGE = %i\n", WSI_TOKEN_HTTP_ACCEPT_LANGUAGE);
  printf("WSI_TOKEN_HTTP_PRAGMA = %i\n", WSI_TOKEN_HTTP_PRAGMA);
  printf("WSI_TOKEN_HTTP_CACHE_CONTROL = %i\n", WSI_TOKEN_HTTP_CACHE_CONTROL);
  printf("WSI_TOKEN_HTTP_AUTHORIZATION = %i\n", WSI_TOKEN_HTTP_AUTHORIZATION);
  printf("WSI_TOKEN_HTTP_COOKIE = %i\n", WSI_TOKEN_HTTP_COOKIE);
  printf("WSI_TOKEN_HTTP_CONTENT_LENGTH = %i\n", WSI_TOKEN_HTTP_CONTENT_LENGTH);
  printf("WSI_TOKEN_HTTP_CONTENT_TYPE = %i\n", WSI_TOKEN_HTTP_CONTENT_TYPE);
  printf("WSI_TOKEN_HTTP_DATE = %i\n", WSI_TOKEN_HTTP_DATE);
  printf("WSI_TOKEN_HTTP_RANGE = %i\n", WSI_TOKEN_HTTP_RANGE);
  printf("WSI_TOKEN_HTTP_REFERER = %i\n", WSI_TOKEN_HTTP_REFERER);
  printf("WSI_TOKEN_KEY = %i\n", WSI_TOKEN_KEY);
  printf("WSI_TOKEN_VERSION = %i\n", WSI_TOKEN_VERSION);
  printf("WSI_TOKEN_SWORIGIN = %i\n", WSI_TOKEN_SWORIGIN);
  printf("WSI_TOKEN_HTTP_COLON_AUTHORITY = %i\n", WSI_TOKEN_HTTP_COLON_AUTHORITY);
  printf("WSI_TOKEN_HTTP_COLON_METHOD = %i\n", WSI_TOKEN_HTTP_COLON_METHOD);
  printf("WSI_TOKEN_HTTP_COLON_PATH = %i\n", WSI_TOKEN_HTTP_COLON_PATH);
  printf("WSI_TOKEN_HTTP_COLON_SCHEME = %i\n", WSI_TOKEN_HTTP_COLON_SCHEME);
  printf("WSI_TOKEN_HTTP_COLON_STATUS = %i\n", WSI_TOKEN_HTTP_COLON_STATUS);
  printf("WSI_TOKEN_HTTP_ACCEPT_CHARSET = %i\n", WSI_TOKEN_HTTP_ACCEPT_CHARSET);
  printf("WSI_TOKEN_HTTP_ACCEPT_RANGES = %i\n", WSI_TOKEN_HTTP_ACCEPT_RANGES);
  printf("WSI_TOKEN_HTTP_ACCESS_CONTROL_ALLOW_ORIGIN = %i\n",
         WSI_TOKEN_HTTP_ACCESS_CONTROL_ALLOW_ORIGIN);
  printf("WSI_TOKEN_HTTP_AGE = %i\n", WSI_TOKEN_HTTP_AGE);
  printf("WSI_TOKEN_HTTP_ALLOW = %i\n", WSI_TOKEN_HTTP_ALLOW);
  printf("WSI_TOKEN_HTTP_CONTENT_DISPOSITION = %i\n", WSI_TOKEN_HTTP_CONTENT_DISPOSITION);
  printf("WSI_TOKEN_HTTP_CONTENT_ENCODING = %i\n", WSI_TOKEN_HTTP_CONTENT_ENCODING);
  printf("WSI_TOKEN_HTTP_CONTENT_LANGUAGE = %i\n", WSI_TOKEN_HTTP_CONTENT_LANGUAGE);
  printf("WSI_TOKEN_HTTP_CONTENT_LOCATION = %i\n", WSI_TOKEN_HTTP_CONTENT_LOCATION);
  printf("WSI_TOKEN_HTTP_CONTENT_RANGE = %i\n", WSI_TOKEN_HTTP_CONTENT_RANGE);
  printf("WSI_TOKEN_HTTP_ETAG = %i\n", WSI_TOKEN_HTTP_ETAG);
  printf("WSI_TOKEN_HTTP_EXPECT = %i\n", WSI_TOKEN_HTTP_EXPECT);
  printf("WSI_TOKEN_HTTP_EXPIRES = %i\n", WSI_TOKEN_HTTP_EXPIRES);
  printf("WSI_TOKEN_HTTP_FROM = %i\n", WSI_TOKEN_HTTP_FROM);
  printf("WSI_TOKEN_HTTP_IF_MATCH = %i\n", WSI_TOKEN_HTTP_IF_MATCH);
  printf("WSI_TOKEN_HTTP_IF_RANGE = %i\n", WSI_TOKEN_HTTP_IF_RANGE);
  printf("WSI_TOKEN_HTTP_IF_UNMODIFIED_SINCE = %i\n", WSI_TOKEN_HTTP_IF_UNMODIFIED_SINCE);
  printf("WSI_TOKEN_HTTP_LAST_MODIFIED = %i\n", WSI_TOKEN_HTTP_LAST_MODIFIED);
  printf("WSI_TOKEN_HTTP_LINK = %i\n", WSI_TOKEN_HTTP_LINK);
  printf("WSI_TOKEN_HTTP_LOCATION = %i\n", WSI_TOKEN_HTTP_LOCATION);
  printf("WSI_TOKEN_HTTP_MAX_FORWARDS = %i\n", WSI_TOKEN_HTTP_MAX_FORWARDS);
  printf("WSI_TOKEN_HTTP_PROXY_AUTHENTICATE = %i\n", WSI_TOKEN_HTTP_PROXY_AUTHENTICATE);
  printf("WSI_TOKEN_HTTP_PROXY_AUTHORIZATION = %i\n", WSI_TOKEN_HTTP_PROXY_AUTHORIZATION);
  printf("WSI_TOKEN_HTTP_REFRESH = %i\n", WSI_TOKEN_HTTP_REFRESH);
  printf("WSI_TOKEN_HTTP_RETRY_AFTER = %i\n", WSI_TOKEN_HTTP_RETRY_AFTER);
  printf("WSI_TOKEN_HTTP_SERVER = %i\n", WSI_TOKEN_HTTP_SERVER);
  printf("WSI_TOKEN_HTTP_SET_COOKIE = %i\n", WSI_TOKEN_HTTP_SET_COOKIE);
  printf("WSI_TOKEN_HTTP_STRICT_TRANSPORT_SECURITY = %i\n",
         WSI_TOKEN_HTTP_STRICT_TRANSPORT_SECURITY);
  printf("WSI_TOKEN_HTTP_TRANSFER_ENCODING = %i\n", WSI_TOKEN_HTTP_TRANSFER_ENCODING);
  printf("WSI_TOKEN_HTTP_USER_AGENT = %i\n", WSI_TOKEN_HTTP_USER_AGENT);
  printf("WSI_TOKEN_HTTP_VARY = %i\n", WSI_TOKEN_HTTP_VARY);
  printf("WSI_TOKEN_HTTP_VIA = %i\n", WSI_TOKEN_HTTP_VIA);
  printf("WSI_TOKEN_HTTP_WWW_AUTHENTICATE = %i\n", WSI_TOKEN_HTTP_WWW_AUTHENTICATE);
  printf("WSI_TOKEN_PATCH_URI = %i\n", WSI_TOKEN_PATCH_URI);
  printf("WSI_TOKEN_PUT_URI = %i\n", WSI_TOKEN_PUT_URI);
  printf("WSI_TOKEN_DELETE_URI = %i\n", WSI_TOKEN_DELETE_URI);
  printf("WSI_TOKEN_HTTP_URI_ARGS = %i\n", WSI_TOKEN_HTTP_URI_ARGS);
  printf("WSI_TOKEN_PROXY = %i\n", WSI_TOKEN_PROXY);
  printf("WSI_TOKEN_HTTP_X_REAL_IP = %i\n", WSI_TOKEN_HTTP_X_REAL_IP);
  printf("WSI_TOKEN_HTTP1_0 = %i\n", WSI_TOKEN_HTTP1_0);
  printf("WSI_TOKEN_X_FORWARDED_FOR = %i\n", WSI_TOKEN_X_FORWARDED_FOR);
  printf("WSI_TOKEN_CONNECT = %i\n", WSI_TOKEN_CONNECT);
  printf("WSI_TOKEN_HEAD_URI = %i\n", WSI_TOKEN_HEAD_URI);
  printf("WSI_TOKEN_TE = %i\n", WSI_TOKEN_TE);
  printf("WSI_TOKEN_REPLAY_NONCE = %i\n", WSI_TOKEN_REPLAY_NONCE);
  printf("WSI_TOKEN_COLON_PROTOCOL = %i\n", WSI_TOKEN_COLON_PROTOCOL);
  printf("WSI_TOKEN_X_AUTH_TOKEN = %i\n", WSI_TOKEN_X_AUTH_TOKEN);
  printf("WSI_TOKEN_DSS_SIGNATURE = %i\n", WSI_TOKEN_DSS_SIGNATURE);
  printf("WSI_TOKEN_COUNT = %i\n", WSI_TOKEN_COUNT);
  printf("WSI_TOKEN_NAME_PART = %i\n", WSI_TOKEN_NAME_PART);
  printf("WSI_TOKEN_UNKNOWN_VALUE_PART = %i\n", WSI_TOKEN_UNKNOWN_VALUE_PART);
  printf("WSI_TOKEN_SKIPPING = %i\n", WSI_TOKEN_SKIPPING);
  printf("WSI_TOKEN_SKIPPING_SAW_CR = %i\n", WSI_TOKEN_SKIPPING_SAW_CR);
  printf("WSI_TOKEN_COUNT = %i\n", WSI_TOKEN_COUNT);
  printf("WSI_TOKEN_HTTP_URI_ARGS = %i\n", WSI_TOKEN_HTTP_URI_ARGS);

  printf("%s = %d\n", "LWS_CALLBACK_ESTABLISHED", LWS_CALLBACK_ESTABLISHED);
  printf("%s = %d\n",
         "LWS_CALLBACK_CLIENT_CONNECTION_ERROR",
         LWS_CALLBACK_CLIENT_CONNECTION_ERROR);
  printf("%s = %d\n",
         "LWS_CALLBACK_CLIENT_FILTER_PRE_ESTABLISH",
         LWS_CALLBACK_CLIENT_FILTER_PRE_ESTABLISH);
  printf("%s = %d\n", "LWS_CALLBACK_CLIENT_ESTABLISHED", LWS_CALLBACK_CLIENT_ESTABLISHED);
  printf("%s = %d\n", "LWS_CALLBACK_CLOSED", LWS_CALLBACK_CLOSED);
  printf("%s = %d\n", "LWS_CALLBACK_CLOSED_HTTP", LWS_CALLBACK_CLOSED_HTTP);
  printf("%s = %d\n", "LWS_CALLBACK_RECEIVE", LWS_CALLBACK_RECEIVE);
  printf("%s = %d\n", "LWS_CALLBACK_RECEIVE_PONG", LWS_CALLBACK_RECEIVE_PONG);
  printf("%s = %d\n", "LWS_CALLBACK_CLIENT_RECEIVE", LWS_CALLBACK_CLIENT_RECEIVE);
  printf("%s = %d\n", "LWS_CALLBACK_CLIENT_RECEIVE_PONG", LWS_CALLBACK_CLIENT_RECEIVE_PONG);
  printf("%s = %d\n", "LWS_CALLBACK_CLIENT_WRITEABLE", LWS_CALLBACK_CLIENT_WRITEABLE);
  printf("%s = %d\n", "LWS_CALLBACK_SERVER_WRITEABLE", LWS_CALLBACK_SERVER_WRITEABLE);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP", LWS_CALLBACK_HTTP);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP_BODY", LWS_CALLBACK_HTTP_BODY);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP_BODY_COMPLETION", LWS_CALLBACK_HTTP_BODY_COMPLETION);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP_FILE_COMPLETION", LWS_CALLBACK_HTTP_FILE_COMPLETION);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP_WRITEABLE", LWS_CALLBACK_HTTP_WRITEABLE);
  printf("%s = %d\n",
         "LWS_CALLBACK_FILTER_NETWORK_CONNECTION",
         LWS_CALLBACK_FILTER_NETWORK_CONNECTION);
  printf("%s = %d\n",
         "LWS_CALLBACK_FILTER_HTTP_CONNECTION",
         LWS_CALLBACK_FILTER_HTTP_CONNECTION);
  printf("%s = %d\n",
         "LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED",
         LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED);
  printf("%s = %d\n",
         "LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION",
         LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION);
  printf("%s = %d\n",
         "LWS_CALLBACK_OPENSSL_LOAD_EXTRA_CLIENT_VERIFY_CERTS",
         LWS_CALLBACK_OPENSSL_LOAD_EXTRA_CLIENT_VERIFY_CERTS);
  printf("%s = %d\n",
         "LWS_CALLBACK_OPENSSL_LOAD_EXTRA_SERVER_VERIFY_CERTS",
         LWS_CALLBACK_OPENSSL_LOAD_EXTRA_SERVER_VERIFY_CERTS);
  printf("%s = %d\n",
         "LWS_CALLBACK_OPENSSL_PERFORM_CLIENT_CERT_VERIFICATION",
         LWS_CALLBACK_OPENSSL_PERFORM_CLIENT_CERT_VERIFICATION);
  printf("%s = %d\n",
         "LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER",
         LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER);
  printf("%s = %d\n",
         "LWS_CALLBACK_CONFIRM_EXTENSION_OKAY",
         LWS_CALLBACK_CONFIRM_EXTENSION_OKAY);
  printf("%s = %d\n",
         "LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED",
         LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED);
  printf("%s = %d\n", "LWS_CALLBACK_PROTOCOL_INIT", LWS_CALLBACK_PROTOCOL_INIT);
  printf("%s = %d\n", "LWS_CALLBACK_PROTOCOL_DESTROY", LWS_CALLBACK_PROTOCOL_DESTROY);
  printf("%s = %d\n", "LWS_CALLBACK_WSI_CREATE", LWS_CALLBACK_WSI_CREATE);
  printf("%s = %d\n", "LWS_CALLBACK_WSI_DESTROY", LWS_CALLBACK_WSI_DESTROY);
  printf("%s = %d\n", "LWS_CALLBACK_GET_THREAD_ID", LWS_CALLBACK_GET_THREAD_ID);
  printf("%s = %d\n", "LWS_CALLBACK_ADD_POLL_FD", LWS_CALLBACK_ADD_POLL_FD);
  printf("%s = %d\n", "LWS_CALLBACK_DEL_POLL_FD", LWS_CALLBACK_DEL_POLL_FD);
  printf("%s = %d\n", "LWS_CALLBACK_CHANGE_MODE_POLL_FD", LWS_CALLBACK_CHANGE_MODE_POLL_FD);
  printf("%s = %d\n", "LWS_CALLBACK_LOCK_POLL", LWS_CALLBACK_LOCK_POLL);
  printf("%s = %d\n", "LWS_CALLBACK_UNLOCK_POLL", LWS_CALLBACK_UNLOCK_POLL);
  // printf("%s = %d\n",       "LWS_CALLBACK_OPENSSL_CONTEXT_REQUIRES_PRIVATE_KEY",
  // LWS_CALLBACK_OPENSSL_CONTEXT_REQUIRES_PRIVATE_KEY);
  printf("%s = %d\n",
         "LWS_CALLBACK_WS_PEER_INITIATED_CLOSE",
         LWS_CALLBACK_WS_PEER_INITIATED_CLOSE);
  printf("%s = %d\n", "LWS_CALLBACK_WS_EXT_DEFAULTS", LWS_CALLBACK_WS_EXT_DEFAULTS);
  printf("%s = %d\n", "LWS_CALLBACK_CGI", LWS_CALLBACK_CGI);
  printf("%s = %d\n", "LWS_CALLBACK_CGI_TERMINATED", LWS_CALLBACK_CGI_TERMINATED);
  printf("%s = %d\n", "LWS_CALLBACK_CGI_STDIN_DATA", LWS_CALLBACK_CGI_STDIN_DATA);
  printf("%s = %d\n", "LWS_CALLBACK_CGI_STDIN_COMPLETED", LWS_CALLBACK_CGI_STDIN_COMPLETED);
  printf("%s = %d\n",
         "LWS_CALLBACK_ESTABLISHED_CLIENT_HTTP",
         LWS_CALLBACK_ESTABLISHED_CLIENT_HTTP);
  printf("%s = %d\n", "LWS_CALLBACK_CLOSED_CLIENT_HTTP", LWS_CALLBACK_CLOSED_CLIENT_HTTP);
  printf("%s = %d\n", "LWS_CALLBACK_RECEIVE_CLIENT_HTTP", LWS_CALLBACK_RECEIVE_CLIENT_HTTP);
  printf("%s = %d\n", "LWS_CALLBACK_COMPLETED_CLIENT_HTTP", LWS_CALLBACK_COMPLETED_CLIENT_HTTP);
  printf("%s = %d\n",
         "LWS_CALLBACK_RECEIVE_CLIENT_HTTP_READ",
         LWS_CALLBACK_RECEIVE_CLIENT_HTTP_READ);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP_BIND_PROTOCOL", LWS_CALLBACK_HTTP_BIND_PROTOCOL);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP_DROP_PROTOCOL", LWS_CALLBACK_HTTP_DROP_PROTOCOL);
  printf("%s = %d\n", "LWS_CALLBACK_CHECK_ACCESS_RIGHTS", LWS_CALLBACK_CHECK_ACCESS_RIGHTS);
  printf("%s = %d\n", "LWS_CALLBACK_PROCESS_HTML", LWS_CALLBACK_PROCESS_HTML);
  printf("%s = %d\n", "LWS_CALLBACK_ADD_HEADERS", LWS_CALLBACK_ADD_HEADERS);
  printf("%s = %d\n", "LWS_CALLBACK_SESSION_INFO", LWS_CALLBACK_SESSION_INFO);
  printf("%s = %d\n", "LWS_CALLBACK_GS_EVENT", LWS_CALLBACK_GS_EVENT);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP_PMO", LWS_CALLBACK_HTTP_PMO);
  printf("%s = %d\n", "LWS_CALLBACK_CLIENT_HTTP_WRITEABLE", LWS_CALLBACK_CLIENT_HTTP_WRITEABLE);
  printf("%s = %d\n",
         "LWS_CALLBACK_OPENSSL_PERFORM_SERVER_CERT_VERIFICATION",
         LWS_CALLBACK_OPENSSL_PERFORM_SERVER_CERT_VERIFICATION);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_RX", LWS_CALLBACK_RAW_RX);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_CLOSE", LWS_CALLBACK_RAW_CLOSE);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_WRITEABLE", LWS_CALLBACK_RAW_WRITEABLE);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_ADOPT", LWS_CALLBACK_RAW_ADOPT);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_ADOPT_FILE", LWS_CALLBACK_RAW_ADOPT_FILE);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_RX_FILE", LWS_CALLBACK_RAW_RX_FILE);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_WRITEABLE_FILE", LWS_CALLBACK_RAW_WRITEABLE_FILE);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_CLOSE_FILE", LWS_CALLBACK_RAW_CLOSE_FILE);
  printf("%s = %d\n", "LWS_CALLBACK_SSL_INFO", LWS_CALLBACK_SSL_INFO);
  printf("%s = %d\n", "LWS_CALLBACK_CHILD_CLOSING", LWS_CALLBACK_CHILD_CLOSING);
  printf("%s = %d\n", "LWS_CALLBACK_CGI_PROCESS_ATTACH", LWS_CALLBACK_CGI_PROCESS_ATTACH);
  printf("%s = %d\n", "LWS_CALLBACK_EVENT_WAIT_CANCELLED", LWS_CALLBACK_EVENT_WAIT_CANCELLED);
  printf("%s = %d\n", "LWS_CALLBACK_VHOST_CERT_AGING", LWS_CALLBACK_VHOST_CERT_AGING);
  printf("%s = %d\n", "LWS_CALLBACK_TIMER", LWS_CALLBACK_TIMER);
  printf("%s = %d\n", "LWS_CALLBACK_VHOST_CERT_UPDATE", LWS_CALLBACK_VHOST_CERT_UPDATE);
  printf("%s = %d\n", "LWS_CALLBACK_CLIENT_CLOSED", LWS_CALLBACK_CLIENT_CLOSED);
  printf("%s = %d\n",
         "LWS_CALLBACK_CLIENT_HTTP_DROP_PROTOCOL",
         LWS_CALLBACK_CLIENT_HTTP_DROP_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_WS_SERVER_BIND_PROTOCOL",
         LWS_CALLBACK_WS_SERVER_BIND_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_WS_SERVER_DROP_PROTOCOL",
         LWS_CALLBACK_WS_SERVER_DROP_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_WS_CLIENT_BIND_PROTOCOL",
         LWS_CALLBACK_WS_CLIENT_BIND_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_WS_CLIENT_DROP_PROTOCOL",
         LWS_CALLBACK_WS_CLIENT_DROP_PROTOCOL);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_SKT_BIND_PROTOCOL", LWS_CALLBACK_RAW_SKT_BIND_PROTOCOL);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_SKT_DROP_PROTOCOL", LWS_CALLBACK_RAW_SKT_DROP_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_RAW_FILE_BIND_PROTOCOL",
         LWS_CALLBACK_RAW_FILE_BIND_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_RAW_FILE_DROP_PROTOCOL",
         LWS_CALLBACK_RAW_FILE_DROP_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_CLIENT_HTTP_BIND_PROTOCOL",
         LWS_CALLBACK_CLIENT_HTTP_BIND_PROTOCOL);
  printf("%s = %d\n", "LWS_CALLBACK_HTTP_CONFIRM_UPGRADE", LWS_CALLBACK_HTTP_CONFIRM_UPGRADE);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_PROXY_CLI_RX", LWS_CALLBACK_RAW_PROXY_CLI_RX);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_PROXY_SRV_RX", LWS_CALLBACK_RAW_PROXY_SRV_RX);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_PROXY_CLI_CLOSE", LWS_CALLBACK_RAW_PROXY_CLI_CLOSE);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_PROXY_SRV_CLOSE", LWS_CALLBACK_RAW_PROXY_SRV_CLOSE);
  printf("%s = %d\n",
         "LWS_CALLBACK_RAW_PROXY_CLI_WRITEABLE",
         LWS_CALLBACK_RAW_PROXY_CLI_WRITEABLE);
  printf("%s = %d\n",
         "LWS_CALLBACK_RAW_PROXY_SRV_WRITEABLE",
         LWS_CALLBACK_RAW_PROXY_SRV_WRITEABLE);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_PROXY_CLI_ADOPT", LWS_CALLBACK_RAW_PROXY_CLI_ADOPT);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_PROXY_SRV_ADOPT", LWS_CALLBACK_RAW_PROXY_SRV_ADOPT);
  printf("%s = %d\n",
         "LWS_CALLBACK_RAW_PROXY_CLI_BIND_PROTOCOL",
         LWS_CALLBACK_RAW_PROXY_CLI_BIND_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_RAW_PROXY_SRV_BIND_PROTOCOL",
         LWS_CALLBACK_RAW_PROXY_SRV_BIND_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_RAW_PROXY_CLI_DROP_PROTOCOL",
         LWS_CALLBACK_RAW_PROXY_CLI_DROP_PROTOCOL);
  printf("%s = %d\n",
         "LWS_CALLBACK_RAW_PROXY_SRV_DROP_PROTOCOL",
         LWS_CALLBACK_RAW_PROXY_SRV_DROP_PROTOCOL);
  printf("%s = %d\n", "LWS_CALLBACK_RAW_CONNECTED", LWS_CALLBACK_RAW_CONNECTED);
  printf("%s = %d\n",
         "LWS_CALLBACK_VERIFY_BASIC_AUTHORIZATION",
         LWS_CALLBACK_VERIFY_BASIC_AUTHORIZATION);
  printf("%s = %d\n", "LWS_CALLBACK_WSI_TX_CREDIT_GET", LWS_CALLBACK_WSI_TX_CREDIT_GET);
  printf("%s = %d\n", "LWS_CALLBACK_CLIENT_HTTP_REDIRECT", LWS_CALLBACK_CLIENT_HTTP_REDIRECT);
  printf("%s = %d\n", "LWS_CALLBACK_CONNECTING", LWS_CALLBACK_CONNECTING);
  printf("%s = %d\n",
         "LWS_CALLBACK_MQTT_NEW_CLIENT_INSTANTIATED",
         LWS_CALLBACK_MQTT_NEW_CLIENT_INSTANTIATED);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_IDLE", LWS_CALLBACK_MQTT_IDLE);
  printf("%s = %d\n",
         "LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED",
         LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_SUBSCRIBED", LWS_CALLBACK_MQTT_SUBSCRIBED);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_CLIENT_WRITEABLE", LWS_CALLBACK_MQTT_CLIENT_WRITEABLE);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_CLIENT_RX", LWS_CALLBACK_MQTT_CLIENT_RX);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_UNSUBSCRIBED", LWS_CALLBACK_MQTT_UNSUBSCRIBED);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_DROP_PROTOCOL", LWS_CALLBACK_MQTT_DROP_PROTOCOL);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_CLIENT_CLOSED", LWS_CALLBACK_MQTT_CLIENT_CLOSED);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_ACK", LWS_CALLBACK_MQTT_ACK);
  printf("%s = %d\n", "LWS_CALLBACK_MQTT_RESEND", LWS_CALLBACK_MQTT_RESEND);
  printf("%s = %d\n",
         "LWS_CALLBACK_MQTT_UNSUBSCRIBE_TIMEOUT",
         LWS_CALLBACK_MQTT_UNSUBSCRIBE_TIMEOUT);
  printf("%s = %d\n", "LWS_CALLBACK_USER", LWS_CALLBACK_USER);
  return 0;
}
