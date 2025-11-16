#pragma once

#include "util/ui/AtomicGraphicsObject.hpp"

class Line : public AtomicGraphicsObject {
public:
    Line(RefSerialData::Tx::GraphicColor color, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t thickness)
        : AtomicGraphicsObject(color),
          x1(x1),
          y1(y1),
          x2(x2),
          y2(y2),
          thickness(thickness) {}

    Line() : Line(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configLine(thickness, x1, y1, x2, y2, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX1 == x1 && prevY1 == y1 && prevX2 == x2 && prevY2 == y2 && prevColor == color); }

    uint16_t x1, y1, x2, y2, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev() {
        prevThickness = thickness;
        prevX1 = x1;
        prevY1 = y1;
        prevX2 = x2;
        prevY2 = y2;
        prevColor = color;
    }

    uint16_t prevThickness, prevX1, prevY1, prevX2, prevY2 = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class UnfilledRectangle : public AtomicGraphicsObject {
public:
    UnfilledRectangle(RefSerialData::Tx::GraphicColor color, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t thickness)
        : AtomicGraphicsObject(color),
          x(x),
          y(y),
          width(width),
          height(height),
          thickness(thickness) {}

    UnfilledRectangle() : UnfilledRectangle(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configRectangle(thickness, x, y, width + x, height + y, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevWidth == width && prevHeight == height && prevColor == color); }

    uint16_t x, y, width, height, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevWidth = width;
        prevHeight = height;
        prevColor = color;
    }

    uint16_t prevThickness, prevX, prevY, prevWidth, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};


class FilledRectangle : public AtomicGraphicsObject {
public:
    // thickness behaves as extra width and height
    FilledRectangle(RefSerialData::Tx::GraphicColor color, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t thickness)
        : AtomicGraphicsObject(color),
          x(x),
          y(y),
          width(width),
          height(height),
          thickness(thickness) {}

    FilledRectangle() : FilledRectangle(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        RefSerialTransmitter::configLine(height+thickness, x-thickness/2, height/2 + y, width + x + thickness/2, height/2 + y, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevWidth == width && prevHeight == height && prevColor == color); }

    uint16_t x, y, width, height, thickness;  // can set this directly, will appear next time drawn

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevWidth = width;
        prevHeight = height;
        prevColor = color;
    }

    uint16_t prevThickness, prevX, prevY, prevWidth, prevHeight = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};



class Arc : public AtomicGraphicsObject {
public:
    Arc(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t rx, uint16_t ry, uint16_t startAngle, uint16_t endAngle, uint16_t thickness)
        : AtomicGraphicsObject(color),
          startAngle(startAngle),
          endAngle(endAngle),
          cx(cx),
          cy(cy),
          rx(rx),
          ry(ry),
          thickness(thickness) {}

    Arc() : Arc(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        if(reachesCenter){
            if(rx>ry){
                uint16_t thicknessAdjusted = ry+thickness/2;
                RefSerialTransmitter::configArc(startAngle, endAngle, thicknessAdjusted, cx, cy, rx-ry/2+thickness/4, thicknessAdjusted/2, graphicData);
            } else {
                uint16_t thicknessAdjusted = rx+thickness/2;
                RefSerialTransmitter::configArc(startAngle, endAngle, thicknessAdjusted, cx, cy, thicknessAdjusted/2, ry-rx/2+thickness/4, graphicData);
            }
        } else {
            RefSerialTransmitter::configArc(startAngle, endAngle, thickness, cx, cy, rx, ry, graphicData);
        }
        setPrev();
    }

    bool needsRedrawn() {
        return !(
            prevThickness == thickness && prevReachesCenter == reachesCenter && prevCx == cx && prevCy == cy && prevRx == rx && prevRy == ry && prevColor == color && prevStartAngle == startAngle && prevEndAngle == endAngle);
    }

    uint16_t startAngle, endAngle;       // can set this directly, will appear next time drawn, 0 is up, positive is clockwise, in degrees
    uint16_t cx, cy, rx, ry, thickness;  // can set this directly, will appear next time drawn
    bool reachesCenter = false;          // can set this directly, will appear next time drawn: if true, this arc becomes a wedge
    
private:
    void setPrev() {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevRx = rx;
        prevRy = ry;
        prevStartAngle = startAngle;
        prevEndAngle = endAngle;
        prevColor = color;
        prevReachesCenter = reachesCenter;
    }

    uint16_t prevThickness, prevCx, prevCy, prevRx, prevRy, prevStartAngle, prevEndAngle = 0;
    bool prevReachesCenter = false;
    RefSerialData::Tx::GraphicColor prevColor;
};


// maybe at some point make it so circles and ellipses are arcs
// the reason why they aren't now is because we don't want rx and ry in a circle
// or start and end angle in either
class UnfilledCircle : public AtomicGraphicsObject {
public:
    UnfilledCircle(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t r, uint16_t thickness) : AtomicGraphicsObject(color), cx(cx), cy(cy), r(r), thickness(thickness) {}

    UnfilledCircle() : UnfilledCircle(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) {
        RefSerialTransmitter::configArc(1, 361, thickness, cx, cy, r, r, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevCx == cx && prevCy == cy && prevR == r && prevColor == color); }

    uint16_t cx, cy, r, thickness;  // can set this directly, will appear next time drawn

protected:
    void setPrev() {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevR = r;
        prevColor = color;
    }

    uint16_t prevThickness, prevCx, prevCy, prevR = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class FilledCircle : public UnfilledCircle {
public:
    FilledCircle(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t r, uint16_t thickness)
        : UnfilledCircle(color, cx, cy, r, thickness) {}

    FilledCircle() : UnfilledCircle(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        uint16_t thicknessAdjusted = r+thickness/2;
        RefSerialTransmitter::configArc(1, 361, thicknessAdjusted, cx, cy, thicknessAdjusted/2, thicknessAdjusted/2, graphicData);
        setPrev();
    }
};


class UnfilledEllipse : public AtomicGraphicsObject {
public:
    UnfilledEllipse(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t rx, uint16_t ry, uint16_t thickness)
        : AtomicGraphicsObject(color),
          cx(cx),
          cy(cy),
          rx(rx),
          ry(ry),
          thickness(thickness) {}

    UnfilledEllipse() : UnfilledEllipse(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData)  {
        RefSerialTransmitter::configArc(1, 361, thickness, cx, cy, rx, ry, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevCx == cx && prevCy == cy && prevRx == rx && prevRy == ry && prevColor == color); }

    uint16_t cx, cy, rx, ry, thickness;  // can set this directly, will appear next time drawn

protected:
    void setPrev() {
        prevThickness = thickness;
        prevCx = cx;
        prevCy = cy;
        prevRx = rx;
        prevRy = ry;
        prevColor = color;
    }

    uint16_t prevThickness, prevCx, prevCy, prevRx, prevRy = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};



class FilledEllipse : public UnfilledEllipse {
public:
    FilledEllipse(RefSerialData::Tx::GraphicColor color, uint16_t cx, uint16_t cy, uint16_t rx, uint16_t ry, uint16_t thickness)
        : UnfilledEllipse(color, cx, cy, rx, ry, thickness) {}

    FilledEllipse() : UnfilledEllipse(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        if(rx>ry){
            uint16_t thicknessAdjusted = ry+thickness/2;
            RefSerialTransmitter::configArc(1, 361, thicknessAdjusted, cx, cy, rx-ry/2+thickness/4, thicknessAdjusted/2, graphicData);
        } else {
            uint16_t thicknessAdjusted = rx+thickness/2;
            RefSerialTransmitter::configArc(1, 361, thicknessAdjusted, cx, cy, thicknessAdjusted/2, ry-rx/2+thickness/4, graphicData);
        }
        setPrev();
    }
};


class LargeCenteredArc : public Arc {
public:
    

    LargeCenteredArc(bool isLeft, uint16_t lane) : Arc(), isLeft(isLeft), lane(lane) {
        cx = UISubsystem::HALF_SCREEN_WIDTH;
        cy = UISubsystem::HALF_SCREEN_HEIGHT;
        setLower(0);
        setHigher(1);
        thickness = THICKNESS;
        rx = SIZE0 - lane*THICKNESS;
        ry = SIZE0 - lane*THICKNESS;
    }

    void setLower(float r){
        if(isLeft){
            startAngle = static_cast<uint16_t>(std::lerp(START_ANGLE_LEFT, END_ANGLE_LEFT, r));
        } else {
            endAngle = static_cast<uint16_t>(std::lerp(END_ANGLE_RIGHT, START_ANGLE_RIGHT, r));
        }

        fixZeroThickness();
    }

    void setHigher(float r){
        if(isLeft){
            endAngle = static_cast<uint16_t>(std::lerp(START_ANGLE_LEFT, END_ANGLE_LEFT, r));
        } else {
            startAngle = static_cast<uint16_t>(std::lerp(END_ANGLE_RIGHT, START_ANGLE_RIGHT, r));
        }

        fixZeroThickness();
    }

    void setIsLeft(bool newIsLeft){
        isLeft = newIsLeft;
    }

private:
    static constexpr uint16_t THICKNESS = 5;      // pixels
    static constexpr uint16_t SIZE0 = 392;        // pixels, makes it so we are just inside the left parenthesis thingy if in lane 0, higher number lanes are further in
    
    static constexpr uint16_t START_ANGLE_LEFT = 227;  // degrees, lines up with the bottom of the left parenthesis thingy that is drawn by default
    static constexpr uint16_t END_ANGLE_LEFT = 313;    // degrees, lines up with the top
    
    static constexpr uint16_t START_ANGLE_RIGHT = START_ANGLE_LEFT-180;  // degrees, lines up with the bottom of the left parenthesis thingy that is drawn by default
    static constexpr uint16_t END_ANGLE_RIGHT = END_ANGLE_LEFT-180;    // degrees, lines up with the top

    bool isLeft;
    uint16_t lane;

    //in the event the start and end angle are the same, hide the arc, not have it be a full circle
    void fixZeroThickness() {
        setHidden(startAngle==endAngle);
    }
};




class TextSizer {
public:
    uint16_t height, x, y = 0;  // can set this directly, will appear next time drawn. When setting x, there is alignment (left/center/right)
    uint16_t width = 0;         // can read this, setting will be in vain, reset next time drawn
    int8_t alignment = -1;      // -1 is left aligned, 0 is center aligned, 1 is right aligned: x location marks the right, center, and left ends of the text, respectively
    
    static constexpr int8_t LEFT_ALIGNED = -1;
    static constexpr int8_t CENTER_ALIGNED = 0;
    static constexpr int8_t RIGHT_ALIGNED = 1;

    TextSizer(uint16_t len) : len(len) {}
    TextSizer(uint16_t len, uint16_t x, uint16_t y, uint16_t height) : height(height), x(x), y(y), len(len) {}

    /* resizes this text to fit within the given rect, ignoring width because it doesn't cut off the contained text */
    void inputRect(UnfilledRectangle* rect) {
        x = rect->x;
        y = rect->y;
        height = rect->height;
        calculateNumbers();  // caller might want to have an updated width
    }

    /* resizes the given rect to bound this text, will include width */
    void outputRect(UnfilledRectangle* rect) {
        calculateNumbers();
        rect->x = x;
        rect->y = y;
        rect->width = width;
        rect->height = height;
    }

    /* call this if you set text and height and want an up to date width */
    void calculateNumbers() {
        fontSize = height;
        calculateWidth();
        textX = x;
        if(alignment == CENTER_ALIGNED) textX-=width/2;
        if(alignment == RIGHT_ALIGNED) textX-=width;
        textY = y + height;
    }

private:
    // empirically tested, ratio ends up being 0.404255319149
    static constexpr uint16_t WIDTH_OFFSET_MULT = 19;
    static constexpr uint16_t WIDTH_OFFSET_DIV = 47;

    //need polymorphism later to get len because width depends on len
    void calculateWidth() { width = fontSize * len - fontSize * WIDTH_OFFSET_MULT / WIDTH_OFFSET_DIV; }

protected:
    uint16_t fontSize, textX, textY = 0;  // can read these, but don't set these, set with setTextNumbers
    uint16_t len = 0;                     // for sending integer 123 or text ABC, len would be 3. Not sure about floats yet, need to test
    
    uint16_t intLen(int32_t n) {
        if (n == 0) return 1;
        if (n < 0) return 1 + intLen(-n);
        return std::floor(std::log10(n) + 1);
    }

    // assumes 1.2 will be shown 1.200; 4 as 4.000; and 1.11111 as 1.111, need to test
    uint16_t floatLen(float n) {
        if (n >= 0 && n < 1) return 5;
        if (n < 0) return 1 + floatLen(-n);
        return std::floor(std::log10(n) + 5);
    }

    // assumes null terminated, also strings longer than 30 will say size 30 because only 30 can be send in one message
    uint16_t stringLen(const char* str) {
        uint16_t r = strlen(str);
        return r < 30 ? r : 30;
    }

    void setLen(uint16_t newLen) {
        len = newLen;
        calculateWidth();
    }
};

class IntegerGraphic : public AtomicGraphicsObject, public TextSizer {
public:
    IntegerGraphic(int32_t newInteger, UnfilledRectangle* rect) : AtomicGraphicsObject(rect->color), TextSizer(intLen(newInteger)), thickness(rect->thickness), integer(newInteger) { inputRect(rect); }

    IntegerGraphic() : IntegerGraphic(RefSerialData::Tx::GraphicColor::WHITE, 0, 0, 0, 0, 1) {};

    IntegerGraphic(RefSerialData::Tx::GraphicColor color, int32_t newInteger, uint16_t x, uint16_t y, uint16_t height, uint16_t thickness)
        : AtomicGraphicsObject(color),
          TextSizer(intLen(newInteger), x, y, height),
          thickness(thickness),
          integer(newInteger) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        setLen(intLen(integer));
        calculateNumbers();
        RefSerialTransmitter::configInteger(fontSize, thickness, textX, textY, integer, graphicData);
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevHeight == height && prevColor == color && prevInteger == integer); }

    uint16_t thickness = 0;
    int32_t integer = 0;

    bool integerChanged() {return prevInteger != integer; }

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevInteger = integer;
        prevColor = color;
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    int32_t prevInteger = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class FloatGraphic : public AtomicGraphicsObject, public TextSizer {
public:
    FloatGraphic(float newFloat, UnfilledRectangle* rect) : AtomicGraphicsObject(rect->color), TextSizer(floatLen(newFloat)), thickness(rect->thickness), _float(newFloat) { inputRect(rect); }

    FloatGraphic(RefSerialData::Tx::GraphicColor color, float newFloat, uint16_t x, uint16_t y, uint16_t height, uint16_t thickness)
        : AtomicGraphicsObject(color),
          TextSizer(floatLen(newFloat), x, y, height),
          thickness(thickness),
          _float(newFloat) {}

    virtual void finishConfigGraphicData(RefSerialData::Tx::GraphicData* graphicData) final {
        setLen(floatLen(_float));
        calculateNumbers();
        // the 3 is decimal precision, changing it does nothing
        RefSerialTransmitter::configFloatingNumber(fontSize, 3, thickness, textX, textY, _float, graphicData);
        
        // probably the part that taproot says is broken: have to handle negative values specially
        if(_float<0){
            graphicData->value = _float*-1000;
            graphicData->value = -graphicData->value;
        }
        setPrev();
    }

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevHeight == height && prevColor == color && prevFloat == _float); }

    uint16_t thickness = 0;
    float _float = 0;

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevFloat = _float;
        prevColor = color;
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    float prevFloat = 0;
    RefSerialData::Tx::GraphicColor prevColor;
};

class StringGraphic : public AtomicGraphicsObject, public TextSizer {
private:
    static constexpr int STRING_SIZE = 31;  // not sure if it should be 30 or 31

public:
    StringGraphic(const char* newString, UnfilledRectangle* rect) : AtomicGraphicsObject(rect->color), TextSizer(stringLen(newString)), thickness(rect->thickness) {
        inputRect(rect);
        setString(newString);
    }

    StringGraphic(RefSerialData::Tx::GraphicColor color, const char* newString, uint16_t x, uint16_t y, uint16_t height, uint16_t thickness)
        : AtomicGraphicsObject(color),
          TextSizer(stringLen(newString), x, y, height),
          thickness(thickness) {
        setString(newString);
    }

    void setString(const char* newString) { 
        strncpy(string, newString, STRING_SIZE);
        calculateNumbers();
    }

    void configCharacterData(RefSerialData::Tx::GraphicCharacterMessage* characterData) final {
        setLen(stringLen(string));
        calculateNumbers();
        configGraphicData(&characterData->graphicData);
        RefSerialTransmitter::configCharacterMsg(fontSize, thickness, textX, textY, string, characterData);
        setPrev();
    }

    // StringGraphics fill the data differently. configGraphicGenerics still needs called, but finishConfigGraphicData shouldn't do anything extra
    void finishConfigGraphicData(__attribute__((unused)) RefSerialData::Tx::GraphicData* graphicData) final {}

    bool needsRedrawn() final { return !(prevThickness == thickness && prevX == x && prevY == y && prevHeight == height && prevColor == color && !std::strncmp(string, oldString, STRING_SIZE)); }

    uint16_t thickness = 0;
    char string[STRING_SIZE];

    bool isStringGraphic() final { return true; }

private:
    void setPrev() {
        prevThickness = thickness;
        prevX = x;
        prevY = y;
        prevHeight = height;
        prevColor = color;
        strncpy(oldString, string, STRING_SIZE);
    }

    uint16_t prevX, prevY, prevHeight, prevThickness = 0;
    char oldString[STRING_SIZE];
    RefSerialData::Tx::GraphicColor prevColor;
};