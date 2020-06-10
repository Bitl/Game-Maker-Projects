#define scr_mdu
/*

        ==!> {md} - Automatic Updater <!==
        
        Please find instructions below on how to use this script. Please NOTE that a user account is required to use this.
        
        __________________________________________
        Usage: 
                scr_mdu(gamename,api)
                
                gamename (string) = The Name of your Game / Application (Maximum 100 characters)
                api (string) = The API key for the download package (generated from the website interface)
             
        __________________________________________
        Advanced Usage:
        
                For the more advanced users there is an option for your game to start with a command line argument when there are no updates or when updates have successfully been aplied
                
                    --
                    No Updates: '0'
                
                    Updates Installed: '1'
                
                    Errors Occured: '2'
                    --
            
                This can be read using gml
                    
                | parameter_string(1) |
                
                To Activate this option please add a third argument to this script with a number 1
                
                eg; scr_mdu(gamename,api,1)
                
            --
                
                You may put this script in the start of your game and it will automaticly check to see if any updates where found or not
                It will return the following: (requires the third argument to be 1 eg; scr_mdu(gamename,api,1)   )
                
                    0 - no updates found
                    1 - updates aplied successfully
                    2 - the updater encountered errors 
        __________________________________________
        Web Interface
        
            The web interface allows you to easily manage your updates and create new ones as well as view statistics about your game. You will need to register an account before you can use the updater                                                                                                                                                                                                  */var mdu_par, mdu_name, mdu_api, mdu_3rd; if !file_exists('mdu.exe') {show_message("Unable to find 'mdu.exe'"); exit;} if file_exists('mdu.new') {file_copy('mdu.exe','mdu.old'); file_delete('mdu.exe'); file_copy('mdu.new','mdu.exe'); file_delete('mdu.new');} mdu_par=parameter_string(1);if mdu_par='0' {return(0); exit;}if mdu_par='1' {return(1); exit;}if mdu_par='2' {return(2); exit;}mdu_name=argument0; mdu_api=argument1; mdu_3rd=argument2; if mdu_3rd=1 {mdu_3rd=' "adv"'}if mdu_name='' or mdu_api='' {show_message('Please Enter valid arguments'); exit;}if secure_mode=true {show_message('Please turn of secure mode'); exit;}execute_program('mdu.exe','"'+string(argument0)+'" "'+string(argument1)+'" "'+string(parameter_string(0))+'"'+string(mdu_3rd),0);game_end();/*
            
        __________________________________________
        Help
        
            If you have any questions or you need some help please visit the web interface or reply in the gmc topic
            
        __________________________________________
        Customization
        
            You can add your own logo into the updater it needs to be an image:
            
            Width: 164
            Height: 72 
            File Format: PNG (Portable Network Graphic)
            Non-Transparent (with a WHITE background)
            
            --
            
            The Image can either be saved as (in the same directory as mdu.exe):
            
            img_mdu.png 
            
            or
            
            img.mdu (just rename it)
            
        __________________________________________
        Commercial Usage
        
           This can be used for commercial usage at no additional cost. Should you however require interface customisations for you game please contact us.
                
        
        ---- > MADE BY {md} - mme < ----

*/

