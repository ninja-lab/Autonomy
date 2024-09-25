

pat1 = "autoNomy";
pat2 = "Exit";
for i = [0:2]
    inp = input("Enter the name of the course: ", "s");
    if contains(inp ,pat1,'IgnoreCase',true)
        disp('Correct')
        return
    elseif contains(inp ,pat2,'IgnoreCase',true)
        return 
    end 
end 

%https://stackoverflow.com/questions/38723138/matlab-execute-script-from-linux-command-line


































