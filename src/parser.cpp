#include "parser.h"



// I know I'm treating namespaces as classes, I beg for your mercy if this is sacrilegious
namespace parser {
    // Serial object for display
    RawSerial pc(SERIAL_TX, SERIAL_RX);

    // Serial Input variables
    static char input_buffer[BUFF_SIZE];
    static volatile int input_counter = 0;
    static volatile bool input_ready = false;

    // Shared resources written by the parser
    float target_pos = 0;
    float target_vel = 0;
    float tune_period = 0.5;
    update_t op_code = OP_NIL;
    int tunes_list[TUNE_BUFFER];
    int durations_list[TUNE_BUFFER];
    volatile bool ready[4] =  {false, false, false, false};//{true, true, true, true};//{false, false, false};

    // This is the mapping from our internal representation of a note
    // to its half-period 
    static const int *note_half_period_map[8];

    // Mapping of half periods of each note
    static const int error[] = {0,0,0};
    
    // Half period mappings for the middle octave
    // static const int A[] = {1204, 1136, 1073};  //A^ , A, A#
    // static const int B[] = {1073, 1012, 0};  	//B^ , B, B#
    // static const int C[] = {0, 1911, 1804};  	//C^ , C, C#
    // static const int D[] = {1804, 1703, 1607};  //D^ , D, D#
    // static const int E[] = {1607, 1517, 0};  	//E^ , E, E#
    // static const int F[] = {0, 1432, 1351};  	//F^ , F, F#
    // static const int G[] = {1351, 1273, 1204};  //G^ , G, G# 

    // Half period mappings for the 8th octave
    static const int A[] = {1204 >> 4, 1136 >> 4, 1073 >> 4};  	//A^ , A, A#
    static const int B[] = {1073 >> 4, 1012 >> 4, 0};  			//B^ , B, B#
    static const int C[] = {0, 1911 >> 4, 1804 >> 4};  			//C^ , C, C#
    static const int D[] = {1804 >> 4, 1703 >> 4, 1607 >> 4};  	//D^ , D, D#
    static const int E[] = {1607 >> 4, 1517 >> 4, 0};  			//E^ , E, E#
    static const int F[] = {0, 1432 >> 4, 1351 >> 4};  			//F^ , F, F#
    static const int G[] = {1351 >> 4, 1273 >> 4, 1204 >> 4};  	//G^ , G, G#


    // Parser Functions

    // Interrupt function called whenever there is a user keystroke
    static void serial_isr(){
        if (pc.readable() && (input_counter < BUFF_SIZE) && (!input_ready)) {
            // Stores keystoke into an internal buffer
            input_buffer[input_counter++] = pc.getc();
            pc.putc(input_buffer[input_counter-1]); 
            
            // Let's parser process the input whenever user presses enter button 
            if (input_buffer[input_counter-1] == '\r') {            
                pc.putc('\n'); 
                input_ready = true;
            }   
        } 
    }

    // Threading implementation for parser
    static int parseCommand(const int _length){
        // Parsing variables
        char op_char = 0;
        char float_buffer[8];
        int note_buffer[3];

        // array indices
        int count = 0;
        int note_count = 0;
        int float_buffer_ind = 0;

        // Update variables, so that previous values will not be overwritten if function encounters an invalid input
        uint8_t new_op = 0;
        float new_pos = 0;
        float new_vel = 0;
        float bpm = 60;
        int new_tunes[TUNE_BUFFER];
        int new_durations[TUNE_BUFFER];

        // Checks if string input is empty (except for '\r')
        if (_length <= 1){
            return 0;
        }
        
        // Initialising buffers and internal variables
        std::memset(float_buffer, 0, 8);
        std::memset(note_buffer, 0, 3);
        std::memset(tunes_list, 0, TUNE_BUFFER * sizeof(int));
        std::memset(durations_list, 0, TUNE_BUFFER * sizeof(int));
        std::memset(new_tunes, 0, TUNE_BUFFER * sizeof(int));
        std::memset(new_durations, 0, TUNE_BUFFER * sizeof(int));
        note_buffer[1] = 1;

        // Iterating through input_buffer
        while (count < _length) {

        	// Reads the current character
            char current_char = input_buffer[count++];
            

            switch(current_char){
                case 'R':
                    if ((op_char == 'V') || (op_char == 'R') || (op_char == 'T') || (op_char == 'M')) {
                        return 0;
                    } else {
                        op_char = current_char; 
                    }
                    break;

                case 'V':
                    if (op_char == 'R') {
                        op_char = 'V';
                        new_pos = (float)std::atof(float_buffer);
                        std::memset(float_buffer,0,8);
                        float_buffer_ind = 0;
                        new_op |= 0x01;

                    } else if ((op_char == 'V') || (op_char == 'T') || (op_char == 'M')){
                        return 0; // For invalid syntax
                    } else {
                        op_char = current_char; 
                    }
                    break;
                
                case 'T':
                    if ((op_char == 'V') || (op_char == 'R') || (op_char == 'M')) {
                        return 0; // For invalid syntax
                    } else {
                        op_char = current_char; 
                    }
                    break;

                case 'M':
                    if ((op_char == 'V') || (op_char == 'R') || (op_char == 'T') ) {
                        return 0; // For invalid syntax
                    } else {
                        op_char = current_char;
                    }
                    break;

                // Stores new values at the end of the buffer 
                case '\r':
                    if(op_char == 'R'){
                        new_pos = (float)std::atof(float_buffer);
                        new_op |= 0x01;

                    } else if (op_char == 'V') {
                        new_vel = (float)std::atof(float_buffer);
                        new_op |= 0x02;

                    } else if (op_char == 'T') {
                        new_op |= 0x04;

                    } else if (op_char == 'M') {
                        bpm = (float)std::atof(float_buffer);
                        new_op = 5;
                    }
                    
                    break;

                // Stores characters in the float/note buffer
                default:

                	// Handling musical notes
                    if (op_char == 'T') {

                    	// if the current character is between 'A' to 'G'
                        if ((current_char == 'A') || (current_char == 'B') || 
                            (current_char == 'C') || (current_char == 'D') || 
                            (current_char == 'E') || (current_char == 'F') || 
                            (current_char == 'G')) {
                            note_buffer[0] = int(current_char - 'A') + 1;
                        }              

                        // Sharp or flat
                        if (current_char == '^'){
                            note_buffer[1] = 0;
                        } else if (current_char == '#') {
                            note_buffer[1] = 2;
                        }

                        // Duration
                        if ((current_char == '1') || (current_char == '2') || 
                            (current_char == '3') || (current_char == '4') || 
                            (current_char == '5') || (current_char == '6') || 
                            (current_char == '7') || ( current_char == '8')) {
                            int reps = int(current_char - '0');

                            int parsed_note = note_half_period_map[note_buffer[0]][note_buffer[1]];
                            
                            if (parsed_note == 0) {
                                return 0; // Invalid note
                            } 

                            // Stores the newly parsed note
                            new_tunes[note_count] = parsed_note;
                            new_durations[note_count++] = reps;

                            // Reset buffers
                            std::memset(note_buffer, 0, 3);
                            note_buffer[1] = 1;
                        }
                    } else {
                    	// Stores it in the value buffer 
                        float_buffer[float_buffer_ind++] = current_char;
                    }
                    break;
            }
        }

        // If the function reaches here, it has parsed the string correctly
        // proceeds to store the parsed values onto the shared resources
        switch (new_op){
            case 1:
            	target_pos = new_pos;
                op_code = OP_POS;
                break;
            case 2:
            	target_vel = new_vel;
                op_code = OP_VEL;
                break;
            case 3:
            	target_pos = new_pos;
            	target_vel = new_vel;
                op_code = OP_PV;
                break;
            case 4:
                std::memcpy(tunes_list, new_tunes, sizeof(tunes_list));
                std::memcpy(durations_list, new_durations, sizeof(durations_list));
                op_code = OP_TUNE;
                break;
            case 5:
            	tune_period = 60 / bpm;
                op_code = OP_BPM;
                break;
            default:
                op_code = OP_NIL;
                break;
        }

        return 1; // Returns a 1 indicating that the parsed string is valid
    }

    // Threading implementation of the parser
    void pollSerialIn(){
        while(1){
        	// If user hits the enter button
            if (input_ready) {
                int length = input_counter;
				
				// Sets the ready variable if the parsed string is valid                
                if(parseCommand(length)){
                    // Unrolling for faster assignment
                    ready[0] = true;
                    ready[1] = true;
                    ready[2] = true;
                    ready[3] = true;

                }

                // Resets the input variables
                std::memset(input_buffer, 0 , BUFF_SIZE);
                input_ready = false;
                input_counter = 0;
            }

            // Long waiting time for fun
            Thread::wait(500);
        }
    }

    // Initialization function for serial and mapping
    void init(){
        pc.baud(9600);
        pc.attach(&serial_isr);
            
        note_half_period_map[0] = error; 
        note_half_period_map[1] = A; 
        note_half_period_map[2] = B; 
        note_half_period_map[3] = C;
        note_half_period_map[4] = D;
        note_half_period_map[5] = E;
        note_half_period_map[6] = F;
        note_half_period_map[7] = G;  
    }
}