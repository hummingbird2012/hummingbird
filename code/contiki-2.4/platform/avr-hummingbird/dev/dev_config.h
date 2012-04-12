/*----------------------------------------------------------------------------
 Copyright:      Radig Ulrich  mailto: mail@ulrichradig.de
 Author:         Radig Ulrich
 Remarks:
 known Problems: none
 Version:        16.11.2008
 Description:    Webserver Config-File

 Dieses Programm ist freie Software. Sie können es unter den Bedingungen der
 GNU General Public License, wie von der Free Software Foundation veröffentlicht,
 weitergeben und/oder modifizieren, entweder gemäß Version 2 der Lizenz oder
 (nach Ihrer Option) jeder späteren Version.

 Die Veröffentlichung dieses Programms erfolgt in der Hoffnung,
 daß es Ihnen von Nutzen sein wird, aber OHNE IRGENDEINE GARANTIE,
 sogar ohne die implizite Garantie der MARKTREIFE oder der VERWENDBARKEIT
 FÜR EINEN BESTIMMTEN ZWECK. Details finden Sie in der GNU General Public License.

 Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 Programm erhalten haben.
 Falls nicht, schreiben Sie an die Free Software Foundation,
 Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
------------------------------------------------------------------------------*/

/**
 * \ingroup main	Hauptprogramm
 *
 */

/**
 * \file
 * Konfigurationsdatei
 *
 * \author Ulrich Radig & W.Wallucks
 */

#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

//Clock speed
//#define F_CPU 16000000UL
//#define F_CPU 14745600UL
//#define F_CPU 11059200UL
#define F_CPU 1000000UL

#define TIMERBASE		25		//!< Time base for timer interrupt is 25 ms

/** USART *************/
#define BAUDRATE 	4800		//!< Baud rate of serial interface
#define USART_USE1		0		//!< USART0 and 0 for 1 for USART1 at ATmega644P

/** 1-Wire ************/

/** RF  ************/
#define USE_TX_MODE  1

#endif //_DEV_CONFIG_H_

/*
 */
