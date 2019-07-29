class ColorMap
{
public:
    ColorMap(double mn, double mx): mn(mn), mx(mx){}
    void setMinMax(double min, double max){ mn = min; mx = max;}
    void setMin(double min){mn = min;}
    void setMax(double max){mx = max;}
    void getColor(double c,uint8_t& R, uint8_t& G, uint8_t& B)
	{
        double normalized = (c - mn)/(mx-mn) * 2 - 1;
        R = (int) (base(normalized - 0.5) * 255);
        G = (int) (base(normalized) * 255);
        B = (int) (base(normalized + 0.5) * 255);
    }
    void getColor(double c, double &rd, double &gd, double &bd)
	{
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(double c){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
    
private:
    double interpolate(double val, double y0, double x0, double y1, double x1){
        return (val - x0)*(y1-y0)/(x1-x0) + y0;
    }
    double base(double val){
        if (val <= -0.75) return 0;
        else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
        else if (val <= 0.25) return 1;
        else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
        else return 0;
    }
private:
    double mn,mx;
};


class Color
{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;

public:
    Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B)
	{

    }

    void getColor(uint8_t &R,uint8_t &G,uint8_t &B)
	{
        R = r;
        G = g;
        B = b;
    }
    void getColor(double &rd, double &gd, double &bd)
	{
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(){
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};