function s = set_default(s, name, value)
%SET_DEFAULT Set default field value in a struct.

    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = value;
    end
end
