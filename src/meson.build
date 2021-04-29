# SPDX-License-Identifier: CC0-1.0

# Cache system paths
libcamera_datadir = get_option('datadir') / 'libcamera'
libcamera_libdir = get_option('libdir') / 'libcamera'
libcamera_sysconfdir = get_option('sysconfdir') / 'libcamera'

libcamera_prefix_datadir = get_option('prefix') / libcamera_datadir
libcamera_prefix_sysconfdir = get_option('prefix') / libcamera_sysconfdir

config_h.set('LIBCAMERA_DATA_DIR', '"' + libcamera_prefix_datadir + '"')
config_h.set('LIBCAMERA_SYSCONF_DIR', '"' + libcamera_prefix_sysconfdir + '"')

summary({
          'LIBCAMERA_DATA_DIR' : libcamera_prefix_datadir,
          'LIBCAMERA_SYSCONF_DIR' : libcamera_prefix_sysconfdir,
        }, section : 'Paths')

# Module Signing
openssl = find_program('openssl', required : true)
if openssl.found()
    ipa_priv_key = custom_target('ipa-priv-key',
                                 output : ['ipa-priv-key.pem'],
                                 command : [gen_ipa_priv_key, '@OUTPUT@'])
    config_h.set('HAVE_IPA_PUBKEY', 1)
    ipa_sign_module = true
else
    ipa_sign_module = false
endif

libcamera_cpp_args = []
libcamera_objects = []

# The 'android' subdir must be processed first, and the build targets
# are included directly into the libcamera library when this is enabled.
subdir('android')

subdir('libcamera')
subdir('ipa')

subdir('lc-compliance')

subdir('cam')
subdir('qcam')

subdir('gstreamer')
subdir('v4l2')
