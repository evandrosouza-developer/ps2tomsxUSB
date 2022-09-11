#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if !defined DBASEMGT_H
#define DBASEMGT_H

/*entry point*/
int flash_rw(void);
void database_setup(void);

#endif	//#if !defined DBASEMGT_H

#ifdef __cplusplus
}
#endif
