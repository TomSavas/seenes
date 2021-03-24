#ifndef SEENES_H
#define SEENES_H

#define SEENES_STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]

#endif
