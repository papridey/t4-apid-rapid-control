function CONST = rapid_constants()
    CONST.SEC_PER_MIN = 60;
    CONST.toMin = @(sec) sec / CONST.SEC_PER_MIN;
end
