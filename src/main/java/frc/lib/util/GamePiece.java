package frc.lib.util;

public class GamePiece {
    public static GamePieceType gamePiece;
    public static enum GamePieceType{
        None, Cone, Cube;
    }

    public static void setGamePiece(GamePieceType gamePieceType){
        gamePiece = gamePieceType;
    }

    public static GamePieceType getGamePiece(){
        return gamePiece;
    }
    public static void toggleGamePiece(){
        if(gamePiece == GamePieceType.Cone){
            gamePiece = GamePieceType.Cube;
        }
        else{
            gamePiece = GamePieceType.Cone;
        }
    }
}
