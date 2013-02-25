/* 
 * File:   globals.h
 * Author: sickboy
 *
 * Created on February 6, 2013, 1:43 PM
 */

#ifndef GLOBALS_RYAN_H
#define	GLOBALS_RYAN_H

#ifdef	__cplusplus
extern "C" {
#endif

#define input_num 5
#define dim 2   /**< Constant value containing the dimension of the state space. */
static const float d0=0.6; /**<Rho_0*/
static const float d1=1; /**<Rho_1*/
static const float d2=1.7; /**<Rho_2*/ 
static const int M=3; /**<Constant value describing the maximum degree condition*/
static const int W=1; /**<Constant value describing the minimum degree condition*/


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBALS_H */

