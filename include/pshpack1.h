/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*
**  $Header: /home/cvs/beta/linuxcan_v4/include/pshpack1.h,v 1.3 2005/12/07 09:04:07 fo Exp $
*/
#if ! (defined(lint) || defined(_lint) || defined(RC_INVOKED))
#  if defined(_MSC_VER)
#    if ( _MSC_VER > 800 ) || defined(_PUSHPOP_SUPPORTED)
#      pragma warning(disable:4103)
#      if !(defined( MIDL_PASS )) || defined( __midl )
#        pragma pack(push)
#      endif
#      pragma pack(1)
#    else
#      pragma warning(disable:4103)
#      pragma pack(1)
#    endif
#  elif defined(__C166__)
#     pragma pack(1)
#  elif defined(__BORLANDC__)
#    if (__BORLANDC__ >= 0x460)
#       pragma nopackwarning
#       pragma pack(push, 1)
#    else
#       pragma option -a1
#    endif

#  else
#       pragma pack (1)
//#    error Unsupported compiler.
#  endif
#endif /* ! (defined(lint) || defined(_lint) || defined(RC_INVOKED)) */
