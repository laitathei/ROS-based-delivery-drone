class Filter_data:

    def __init__(self):
        self.previous_output = 0
        self.previous_input = 0
    
    def smoothed_data(self,data):
        output = 0.828*self.previous_output + 0.0861*data + 0.0861*self.previous_output
        self.previous_output = output
        self.previous_input = data

        return output

def main():
    return



if __name__== "__main__":
    main()