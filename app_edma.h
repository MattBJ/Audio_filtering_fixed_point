#ifndef app_edma_h_
#define app_edma_h_

/* Create the EDMA configuration structure for ping_rx transfers */
EDMA_Config cfgEdmaPing_rx = {  
  EDMA_OPT_RMK(          /* Making Options parameter register - EDMA_OPT    */ 
  
   EDMA_OPT_PRI_HIGH,    /* Priority levels for EDMA events:
                             EDMA_OPT_PRI_LOW  - Low priority EDMA transfer
                             EDMA_OPT_PRI_HIGH - High priority EDMA transfer*/
   
   EDMA_OPT_ESIZE_16BIT,/* Element size :-
                             EDMA_OPT_ESIZE_32BIT - 32 bit word 
                             EDMA_OPT_ESIZE_16BIT - 16 bit word 
                             EDMA_OPT_ESIZE_8BIT  -  8 bit word              */
   
   EDMA_OPT_2DS_NO,     /* Source dimension :-
                             EDMA_OPT_2DS_NO  - 1-dimensional source    
                             EDMA_OPT_2DS_YES - 2-dimensional source         */  
   
   EDMA_OPT_SUM_NONE,   /* Source address update mode :-           
                             EDMA_OPT_SUM_NONE - Fixed address mode 
                             EDMA_OPT_SUM_INC  - Increment address mode
                             EDMA_OPT_SUM_DEC  - Decrement address mode
                             EDMA_OPT_SUM_IDX  - Address modified by element 
                                                 index or frame Index        */
    EDMA_OPT_2DD_NO,     /* Destination dimension :-  
                             EDMA_OPT_2DD_NO   - 1-dimensional source            
                             EDMA_OPT_2DD_YES  - 2-dimensional source        */
   
    EDMA_OPT_DUM_IDX,    /* Destination address update mode :-                       
                             EDMA_OPT_DUM_NONE - Fixed address mode            
                             EDMA_OPT_DUM_INC  - Increment address mode        
                             EDMA_OPT_DUM_DEC  - Decrement address mode       
                             EDMA_OPT_DUM_IDX  - Address modified by element   
                                                 index or frame Index        */
    
    EDMA_OPT_TCINT_YES,  /* Transfer complete interrupt :-
                             EDMA_OPT_TCINT_NO  - Indication disabled      
                             EDMA_OPT_TCINT_YES - Indication enabled         */  
    
    EDMA_OPT_TCC_OF(0xF),/* Transfer complete code                     */
    
    EDMA_OPT_LINK_YES,   /* Linking of event parameters
                             EDMA_OPT_LINK_NO   -  Disabled 
                             EDMA_OPT_LINK_YES  -  Enabled                   */    
    
    EDMA_OPT_FS_NO       /* Frame synchronization
                             EDMA_OPT_FS_NO  - Channel is element/array 
                                                synchronized 
                             EDMA_OPT_FS_YES -  Channel is frame synchronized*/
  ),
  EDMA_SRC_OF(0x34000000u),/* Source address register
                               MCBSP - ource address                */
  
  EDMA_CNT_OF(0x7F0002),   /* Transfer count parameter
                                BUFF_SZ      - buffer sizes in # of ints     */
                                // 0x80 frames of 2 elements
  	  	  	  	  	  	  	  	// BUFFLEN, sizeof(short)
  
  EDMA_DST_OF(rx_ping),      /* Destination address parameter
                                 ping - destination address                  */ 
  
  EDMA_IDX_OF(0xFF020100),/* Index parameter                                 */
                                // back up(-) 0x3E, advance(+) 0x40, --> advance twice as buffer length, back up same - 1
  	  	  	  	  	  	  	  	// -(BUFFLEN-1)*sizeof(short), BUFFLEN*sizeof(short),
  EDMA_RLD_OF(0x00) /* Count reload/link parameter                     */
};


/* Create the EDMA configuration structure for pong_rx transfers */
EDMA_Config cfgEdmaPong_rx = {  
  EDMA_OPT_RMK(          /* Making Options parameter register - EDMA_OPT    */ 
  
   EDMA_OPT_PRI_HIGH,    /* Priority levels for EDMA events:
                             EDMA_OPT_PRI_LOW  - Low priority EDMA transfer
                             EDMA_OPT_PRI_HIGH - High priority EDMA transfer*/
   
   EDMA_OPT_ESIZE_16BIT,/* Element size :-
                             EDMA_OPT_ESIZE_32BIT - 32 bit word 
                             EDMA_OPT_ESIZE_16BIT - 16 bit word 
                             EDMA_OPT_ESIZE_8BIT  -  8 bit word              */
   
   EDMA_OPT_2DS_NO,     /* Source dimension :-
                             EDMA_OPT_2DS_NO  - 1-dimensional source    
                             EDMA_OPT_2DS_YES - 2-dimensional source         */  
   
   EDMA_OPT_SUM_NONE,   /* Source address update mode :-           
                             EDMA_OPT_SUM_NONE - Fixed address mode 
                             EDMA_OPT_SUM_INC  - Increment address mode
                             EDMA_OPT_SUM_DEC  - Decrement address mode
                             EDMA_OPT_SUM_IDX  - Address modified by element 
                                                 index or frame Index        */
    EDMA_OPT_2DD_NO,     /* Destination dimension :-  
                             EDMA_OPT_2DD_NO   - 1-dimensional source            
                             EDMA_OPT_2DD_YES  - 2-dimensional source        */
   
    EDMA_OPT_DUM_IDX,    /* Destination address update mode :-                       
                             EDMA_OPT_DUM_NONE - Fixed address mode            
                             EDMA_OPT_DUM_INC  - Increment address mode        
                             EDMA_OPT_DUM_DEC  - Decrement address mode       
                             EDMA_OPT_DUM_IDX  - Address modified by element   
                                                 index or frame Index        */
    
    EDMA_OPT_TCINT_YES,  /* Transfer complete interrupt :-
                             EDMA_OPT_TCINT_NO  - Indication disabled      
                             EDMA_OPT_TCINT_YES - Indication enabled         */  
    
    EDMA_OPT_TCC_OF(0xD),/* Transfer complete code                     */
    
    EDMA_OPT_LINK_YES,   /* Linking of event parameters
                             EDMA_OPT_LINK_NO   -  Disabled 
                             EDMA_OPT_LINK_YES  -  Enabled                   */    
    
    EDMA_OPT_FS_NO       /* Frame synchronization
                             EDMA_OPT_FS_NO  - Channel is element/array 
                                                synchronized 
                             EDMA_OPT_FS_YES -  Channel is frame synchronized*/
  ),
  EDMA_SRC_OF(0x34000000u),/* Source address register
                               MCPSP    - source address                */
  
  EDMA_CNT_OF(0x7F0002),   /* Transfer count parameter
                                BUFF_SZ      - buffer sizes in # of ints     */      
  
  EDMA_DST_OF(rx_pong),      /* Destination address parameter
                                 ping - destination address                  */ 
  
  EDMA_IDX_OF(0xFF020100),/* Index parameter                                 */
  
  EDMA_RLD_OF(0x00) /* Count reload/link parameter                     */
};

/* Create the EDMA configuration structure for pang_rx transfers */
EDMA_Config cfgEdmaPang_rx = {
  EDMA_OPT_RMK(          /* Making Options parameter register - EDMA_OPT    */

   EDMA_OPT_PRI_HIGH,    /* Priority levels for EDMA events:
                             EDMA_OPT_PRI_LOW  - Low priority EDMA transfer
                             EDMA_OPT_PRI_HIGH - High priority EDMA transfer*/

   EDMA_OPT_ESIZE_16BIT,/* Element size :-
                             EDMA_OPT_ESIZE_32BIT - 32 bit word
                             EDMA_OPT_ESIZE_16BIT - 16 bit word
                             EDMA_OPT_ESIZE_8BIT  -  8 bit word              */

   EDMA_OPT_2DS_NO,     /* Source dimension :-
                             EDMA_OPT_2DS_NO  - 1-dimensional source
                             EDMA_OPT_2DS_YES - 2-dimensional source         */

   EDMA_OPT_SUM_NONE,   /* Source address update mode :-
                             EDMA_OPT_SUM_NONE - Fixed address mode
                             EDMA_OPT_SUM_INC  - Increment address mode
                             EDMA_OPT_SUM_DEC  - Decrement address mode
                             EDMA_OPT_SUM_IDX  - Address modified by element
                                                 index or frame Index        */
    EDMA_OPT_2DD_NO,     /* Destination dimension :-
                             EDMA_OPT_2DD_NO   - 1-dimensional source
                             EDMA_OPT_2DD_YES  - 2-dimensional source        */

    EDMA_OPT_DUM_IDX,    /* Destination address update mode :-
                             EDMA_OPT_DUM_NONE - Fixed address mode
                             EDMA_OPT_DUM_INC  - Increment address mode
                             EDMA_OPT_DUM_DEC  - Decrement address mode
                             EDMA_OPT_DUM_IDX  - Address modified by element
                                                 index or frame Index        */

    EDMA_OPT_TCINT_YES,  /* Transfer complete interrupt :-
                             EDMA_OPT_TCINT_NO  - Indication disabled
                             EDMA_OPT_TCINT_YES - Indication enabled         */

    EDMA_OPT_TCC_OF(0xB),/* Transfer complete code                     */

    EDMA_OPT_LINK_YES,   /* Linking of event parameters
                             EDMA_OPT_LINK_NO   -  Disabled
                             EDMA_OPT_LINK_YES  -  Enabled                   */

    EDMA_OPT_FS_NO       /* Frame synchronization
                             EDMA_OPT_FS_NO  - Channel is element/array
                                                synchronized
                             EDMA_OPT_FS_YES -  Channel is frame synchronized*/
  ),
  EDMA_SRC_OF(0x34000000u),/* Source address register
                               MCBSP - ource address                */

  EDMA_CNT_OF(0x7F0002),   /* Transfer count parameter
                                BUFF_SZ      - buffer sizes in # of ints     */
                                // 0x20 frames of 2 elements
  	  	  	  	  	  	  	  	// BUFFLEN, sizeof(short)

  EDMA_DST_OF(rx_pang),      /* Destination address parameter
                                 ping - destination address                  */

  EDMA_IDX_OF(0xFF020100),/* Index parameter                                 */
                                // back up(-) 0x3E, advance(+) 0x40,
  	  	  	  	  	  	  	  	// -(BUFFLEN-1)*sizeof(short), BUFFLEN*sizeof(short),
  EDMA_RLD_OF(0x00) /* Count reload/link parameter                     */
};


/* Create the EDMA configuration structure for ping_tx transfers */
EDMA_Config cfgEdmaPing_tx = {  
  EDMA_OPT_RMK(          /* Making Options parameter register - EDMA_OPT    */ 
  
   EDMA_OPT_PRI_HIGH,    /* Priority levels for EDMA events:
                             EDMA_OPT_PRI_LOW  - Low priority EDMA transfer
                             EDMA_OPT_PRI_HIGH - High priority EDMA transfer*/
   
   EDMA_OPT_ESIZE_16BIT,/* Element size :-
                             EDMA_OPT_ESIZE_32BIT - 32 bit word 
                             EDMA_OPT_ESIZE_16BIT - 16 bit word 
                             EDMA_OPT_ESIZE_8BIT  -  8 bit word              */
   
   EDMA_OPT_2DS_NO,     /* Source dimension :-
                             EDMA_OPT_2DS_NO  - 1-dimensional source    
                             EDMA_OPT_2DS_YES - 2-dimensional source         */  
   
   EDMA_OPT_SUM_IDX,   /* Source address update mode :-           
                             EDMA_OPT_SUM_NONE - Fixed address mode 
                             EDMA_OPT_SUM_INC  - Increment address mode
                             EDMA_OPT_SUM_DEC  - Decrement address mode
                             EDMA_OPT_SUM_IDX  - Address modified by element 
                                                 index or frame Index        */
    EDMA_OPT_2DD_NO,     /* Destination dimension :-  
                             EDMA_OPT_2DD_NO   - 1-dimensional source            
                             EDMA_OPT_2DD_YES  - 2-dimensional source        */
   
    EDMA_OPT_DUM_NONE,    /* Destination address update mode :-                       
                             EDMA_OPT_DUM_NONE - Fixed address mode            
                             EDMA_OPT_DUM_INC  - Increment address mode        
                             EDMA_OPT_DUM_DEC  - Decrement address mode       
                             EDMA_OPT_DUM_IDX  - Address modified by element   
                                                 index or frame Index        */
    
    EDMA_OPT_TCINT_YES,  /* Transfer complete interrupt :-
                             EDMA_OPT_TCINT_NO  - Indication disabled      
                             EDMA_OPT_TCINT_YES - Indication enabled         */  
    
    EDMA_OPT_TCC_OF(0xE),/* Transfer complete code                     */
    
    EDMA_OPT_LINK_YES,   /* Linking of event parameters
                             EDMA_OPT_LINK_NO   -  Disabled 
                             EDMA_OPT_LINK_YES  -  Enabled                   */    
    
    EDMA_OPT_FS_NO       /* Frame synchronization
                             EDMA_OPT_FS_NO  - Channel is element/array 
                                                synchronized 
                             EDMA_OPT_FS_YES -  Channel is frame synchronized*/
  ),
  EDMA_SRC_OF(tx_ping),/* Source address register 
                               &ping_data    - source address                */
  
  EDMA_CNT_OF(0x7F0002),   /* Transfer count parameter
                                BUFF_SZ      - buffer sizes in # of ints     */      
  
  EDMA_DST_OF(0x34000000u),      /* Destination address parameter
                                 MCBSP - destination address                  */
  
  EDMA_IDX_OF(0xFF020100),/* Index parameter                                 */
  
  EDMA_RLD_OF(0x00) /* Count reload/link parameter                     */
};


/* Create the EDMA configuration structure for pong_tx transfers */
EDMA_Config cfgEdmaPong_tx = {  
  EDMA_OPT_RMK(          /* Making Options parameter register - EDMA_OPT    */ 
  
   EDMA_OPT_PRI_HIGH,    /* Priority levels for EDMA events:
                             EDMA_OPT_PRI_LOW  - Low priority EDMA transfer
                             EDMA_OPT_PRI_HIGH - High priority EDMA transfer*/
   
   EDMA_OPT_ESIZE_16BIT,/* Element size :-
                             EDMA_OPT_ESIZE_32BIT - 32 bit word 
                             EDMA_OPT_ESIZE_16BIT - 16 bit word 
                             EDMA_OPT_ESIZE_8BIT  -  8 bit word              */
   
   EDMA_OPT_2DS_NO,     /* Source dimension :-
                             EDMA_OPT_2DS_NO  - 1-dimensional source    
                             EDMA_OPT_2DS_YES - 2-dimensional source         */  
   
   EDMA_OPT_SUM_IDX,   /* Source address update mode :-           
                             EDMA_OPT_SUM_NONE - Fixed address mode 
                             EDMA_OPT_SUM_INC  - Increment address mode
                             EDMA_OPT_SUM_DEC  - Decrement address mode
                             EDMA_OPT_SUM_IDX  - Address modified by element 
                                                 index or frame Index        */
    EDMA_OPT_2DD_NO,     /* Destination dimension :-  
                             EDMA_OPT_2DD_NO   - 1-dimensional source            
                             EDMA_OPT_2DD_YES  - 2-dimensional source        */
   
    EDMA_OPT_DUM_NONE,    /* Destination address update mode :-                       
                             EDMA_OPT_DUM_NONE - Fixed address mode            
                             EDMA_OPT_DUM_INC  - Increment address mode        
                             EDMA_OPT_DUM_DEC  - Decrement address mode       
                             EDMA_OPT_DUM_IDX  - Address modified by element   
                                                 index or frame Index        */
    
    EDMA_OPT_TCINT_YES,  /* Transfer complete interrupt :-
                             EDMA_OPT_TCINT_NO  - Indication disabled      
                             EDMA_OPT_TCINT_YES - Indication enabled         */  
    
    EDMA_OPT_TCC_OF(0xC),/* Transfer complete code                     */
    
    EDMA_OPT_LINK_YES,   /* Linking of event parameters
                             EDMA_OPT_LINK_NO   -  Disabled 
                             EDMA_OPT_LINK_YES  -  Enabled                   */    
    
    EDMA_OPT_FS_NO       /* Frame synchronization
                             EDMA_OPT_FS_NO  - Channel is element/array 
                                                synchronized 
                             EDMA_OPT_FS_YES -  Channel is frame synchronized*/
  ),
  EDMA_SRC_OF(tx_pong),/* Source address register 
                               &ping_data    - source address                */
  
  EDMA_CNT_OF(0x7F0002),   /* Transfer count parameter
                                BUFF_SZ      - buffer sizes in # of ints     */      
  
  EDMA_DST_OF(0x34000000u),      /* Destination address parameter
                                 MCBSP - destination address                  */
  
  EDMA_IDX_OF(0xFF020100),/* Index parameter                                 */
  
  EDMA_RLD_OF(0x00) /* Count reload/link parameter                     */
};

/* Create the EDMA configuration structure for pang_tx transfers */
EDMA_Config cfgEdmaPang_tx = {
  EDMA_OPT_RMK(          /* Making Options parameter register - EDMA_OPT    */

   EDMA_OPT_PRI_HIGH,    /* Priority levels for EDMA events:
                             EDMA_OPT_PRI_LOW  - Low priority EDMA transfer
                             EDMA_OPT_PRI_HIGH - High priority EDMA transfer*/

   EDMA_OPT_ESIZE_16BIT,/* Element size :-
                             EDMA_OPT_ESIZE_32BIT - 32 bit word
                             EDMA_OPT_ESIZE_16BIT - 16 bit word
                             EDMA_OPT_ESIZE_8BIT  -  8 bit word              */

   EDMA_OPT_2DS_NO,     /* Source dimension :-
                             EDMA_OPT_2DS_NO  - 1-dimensional source
                             EDMA_OPT_2DS_YES - 2-dimensional source         */

   EDMA_OPT_SUM_IDX,   /* Source address update mode :-
                             EDMA_OPT_SUM_NONE - Fixed address mode
                             EDMA_OPT_SUM_INC  - Increment address mode
                             EDMA_OPT_SUM_DEC  - Decrement address mode
                             EDMA_OPT_SUM_IDX  - Address modified by element
                                                 index or frame Index        */
    EDMA_OPT_2DD_NO,     /* Destination dimension :-
                             EDMA_OPT_2DD_NO   - 1-dimensional source
                             EDMA_OPT_2DD_YES  - 2-dimensional source        */

    EDMA_OPT_DUM_NONE,    /* Destination address update mode :-
                             EDMA_OPT_DUM_NONE - Fixed address mode
                             EDMA_OPT_DUM_INC  - Increment address mode
                             EDMA_OPT_DUM_DEC  - Decrement address mode
                             EDMA_OPT_DUM_IDX  - Address modified by element
                                                 index or frame Index        */

    EDMA_OPT_TCINT_YES,  /* Transfer complete interrupt :-
                             EDMA_OPT_TCINT_NO  - Indication disabled
                             EDMA_OPT_TCINT_YES - Indication enabled         */

    EDMA_OPT_TCC_OF(0xA),/* Transfer complete code                     */

    EDMA_OPT_LINK_YES,   /* Linking of event parameters
                             EDMA_OPT_LINK_NO   -  Disabled
                             EDMA_OPT_LINK_YES  -  Enabled                   */

    EDMA_OPT_FS_NO       /* Frame synchronization
                             EDMA_OPT_FS_NO  - Channel is element/array
                                                synchronized
                             EDMA_OPT_FS_YES -  Channel is frame synchronized*/
  ),
  EDMA_SRC_OF(tx_pang),/* Source address register
                               &ping_data    - source address                */

  EDMA_CNT_OF(0x7F0002),   /* Transfer count parameter
                                BUFF_SZ      - buffer sizes in # of ints     */

  EDMA_DST_OF(0x34000000u),      /* Destination address parameter
                                 MCBSP - destination address                  */

  EDMA_IDX_OF(0xFF020100),/* Index parameter                                 */

  EDMA_RLD_OF(0x00) /* Count reload/link parameter                     */
};


#endif /*app_edma_h_*/
