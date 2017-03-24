#include "parser.h"



// I know I'm treating namespaces as classes, I beg for your mercy if this is sacrilegious
namespace parser {
    // Serial object
    RawSerial pc(SERIAL_TX, SERIAL_RX);

    // Serial Input variables
    static char input_buffer[BUFF_SIZE];
    static volatile int input_counter = 0;
    static volatile bool input_ready = false;

    // Parsed Output to be used by other parts of the code
    float target_pos = 0;
    float target_vel = 0;
    float tune_period = 0.5;
    update_t op_code = OP_NIL;
    int tunes_list[TUNE_BUFFER];
    volatile bool ready[4] =  {false, false, false, false};//{true, true, true, true};//{false, false, false};

    // For Tune setting
    static const int *note_half_period_map[8];

    // Mapping of half periods of each note
    static const int error[] = {0,0,0};
    static const int A[] = {1204, 1136, 1073};  //A^ , A, A#
    static const int B[] = {1073, 1012, 0};  //B^ , B, B#
    static const int C[] = {0, 1911, 1804};  //C^ , C, C#
    static const int D[] = {1804, 1703, 1607};  //D^ , D, D#
    static const int E[] = {1607, 1517, 0};  //E^ , E, E#
    static const int F[] = {0, 1432, 1351};  //F^ , F, F#
    static const int G[] = {1351, 1273, 1204};  //G^ , G, G#


    // Parser Functions
    static void serial_isr(){
        if (pc.readable() && (input_counter < BUFF_SIZE) && (!input_ready)) {
            input_buffer[input_counter++] = pc.getc();
            pc.putc(input_buffer[input_counter-1]); 
            
            if (input_buffer[input_counter-1] == '\r') {            
                pc.putc('\n'); 
                input_ready = true;
            }   
        } 
    }

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

        // Checks if string input is empty (except for '\r')
        if (_length <= 1){
            return 0;
        }
        
        // Initialising buffers
        std::memset(float_buffer, 0, 8);
        std::memset(note_buffer, 0, 3);
        std::memset(tunes_list, 0, TUNE_BUFFER * sizeof(int));
        note_buffer[1] = 1;

        // Iterating through input_buffer
        while (count < _length) {
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
                        return 0;
                    } else {
                        op_char = current_char; 
                    }
                    break;
                
                case 'T':
                    if ((op_char == 'V') || (op_char == 'R') || (op_char == 'M')) {
                        return 0;
                    } else {
                        op_char = current_char; 
                    }
                    break;

                case 'M':
                    if ((op_char == 'V') || (op_char == 'R') || (op_char == 'T') ) {
                        return 0;
                    } else {
                        op_char = current_char;
                    }
                    break;

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

                default:
                    if (op_char == 'T') {
                        if ((current_char == 'A') || (current_char == 'B') || 
                            (current_char == 'C') || (current_char == 'D') || 
                            (current_char == 'E') || (current_char == 'F') || 
                            (current_char == 'G')) {
                            note_buffer[0] = int(current_char - 'A') + 1;
                        }              
                        if (current_char == '^'){
                            note_buffer[1] = 0;
                        } else if (current_char == '#') {
                            note_buffer[1] = 2;
                        }

                        if ((current_char == '1') || (current_char == '2') || 
                            (current_char == '3') || (current_char == '4') || 
                            (current_char == '5') || (current_char == '6') || 
                            (current_char == '7') || ( current_char == '8')) {
                            int reps = int(current_char - '0');

                            int parsed_note = note_half_period_map[note_buffer[0]][note_buffer[1]];
                            
                            if (parsed_note == 0) {
                                // pc.printf("Invalid note\n\r");
                                return 0;
                            } 
                            
                            // if (note_buffer[1] != 1){
                            //  pc.printf("parsed note: %c%c, duration: %d, half_period: %d\n\r", 'A' + note_buffer[0] - 1, temp_char, reps, parsed_note);
                            //  char temp_char = (note_buffer[1] == 0) ? '^' : '#';
                            // } else {
                            //  pc.printf("parsed note: %c, duration: %d, half_period: %d\n\r", 'A' + note_buffer[0] - 1, reps, parsed_note);               
                            // }

                            for (int i = 0; i < reps; i++){
                                new_tunes[note_count++] = parsed_note;
                            }

                            // Reset buffers
                            std::memset(note_buffer, 0, 3);
                            note_buffer[1] = 1;
                        }
                    } else {
                        float_buffer[float_buffer_ind++] = current_char;
                    }
                    break;
            }
        }

        switch (new_op){
            case 1:
                op_code = OP_POS;
                break;
            case 2:
                op_code = OP_VEL;
                break;
            case 3:
                op_code = OP_PV;
                break;
            case 4:
                op_code = OP_TUNE;
                break;
            case 5:
                op_code = OP_BPM;
                break;
            default:
                op_code = OP_NIL;
                break;
        }

        // Update state

        if (op_code != OP_NIL){
            target_pos = new_pos;
            target_vel = new_vel;
            tune_period = 60 / bpm;
            for (int i = 0; i < 16; i++) {
                tunes_list[i] = new_tunes[i];
            }
        }

        return 1;
    }

    void pollSerialIn(){
        while(1){
            if (input_ready) {
                int length = input_counter;
                
                // pc.printf("strlen: %d, echo: %s\n", length, input_buffer); 
                
                if(parseCommand(length)){
                    // Unrolling for faster assignment
                    ready[0] = true;
                    ready[1] = true;
                    ready[2] = true;

                }

                std::memset(input_buffer, 0 , BUFF_SIZE);
                input_ready = false;
                input_counter = 0;
            }

            Thread::wait(500);
        }
    }

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