      SUBROUTINE XERDMP                                                 XER   10
C
C     ABSTRACT
C        XERDMP PRINTS AN ERROR TABLE SHOWING ALL ERRORS WHICH
C        HAVE OCCURRED DURING THE CURRENT EXECUTION, OR SINCE XERDMP
C        WAS LAST CALLED.  AFTER PRINTING, THE ERROR TABLE IS CLEARED,
C        AND IF PROGRAM EXECUTION IS CONTINUED ACCUMULATION OF THE
C        ERROR TABLE BEGINS AT ZERO.
C
C     WRITTEN BY RON JONES, WITH SLATEC COMMON MATH LIBRARY SUBCOMMITTEE
C END OF ABSTRACT
C     LATEST REVISION ---  7 JUNE 1978
C
      CALL XERSAV(1H ,0,0,0,KOUNT)
      RETURN
      END
