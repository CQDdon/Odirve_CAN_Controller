
def pad_list(data_bytes, target_length=8, pad_value=0):
    # Calculate how many elements to pad
    padding_needed = target_length - len(data_bytes)
    
    if padding_needed > 0:
        # Pad the list with the specified pad_value
        data_bytes += [pad_value] * padding_needed  # Append the padding values
    elif padding_needed < 0:
        # Trim the list if it's longer than the target_length
        data_bytes = data_bytes[:target_length]
    
    return data_bytes

data_bytes = [0x00,0x00,0x41,0x20]
data_bytes = pad_list(data_bytes)
print(data_bytes)